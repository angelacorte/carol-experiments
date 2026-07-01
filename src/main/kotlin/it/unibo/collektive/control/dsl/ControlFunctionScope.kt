package it.unibo.collektive.control.dsl

import it.unibo.collektive.control.dsl.expressions.AffineExpression
import it.unibo.collektive.control.dsl.expressions.DecisionVector
import it.unibo.collektive.control.dsl.expressions.RuntimeScalar
import it.unibo.collektive.control.dsl.expressions.VectorExpression
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Device
import it.unibo.collektive.solver.gurobi.GRBVector

/**
 * Formula-side view of one robot endpoint.
 *
 * The endpoint combines a Gurobi decision vector, exposed as [u], with state-dependent quantities
 * exposed as runtime expressions.  Reading [position], [safeMargin], or [maxSpeed] does not capture
 * the value available during installation; instead, it creates an expression evaluated during each
 * constraint update.
 *
 * @property u decision vector associated with this endpoint.
 * @property position current endpoint position as a runtime vector.
 * @property safeMargin current safety radius contribution.
 * @property maxSpeed current speed bound.
 */
class AgentExpression internal constructor(decision: GRBVector, private val device: (FormulaRuntime) -> Device) {
    val u: DecisionVector = DecisionVector(decision)

    val position: VectorExpression = vector { it.position.toDoubleArray() }

    val safeMargin: RuntimeScalar = scalar(Device::safeMargin)

    val maxSpeed: RuntimeScalar = scalar(Device::maxSpeed)

    private fun scalar(read: (Device) -> Double): RuntimeScalar =
        RuntimeScalar { runtime -> read(device(runtime)) }

    private fun vector(read: (Device) -> DoubleArray): VectorExpression =
        VectorExpression { runtime -> read(device(runtime)) }
}

/**
 * Receiver used by CBF and CLF implementations to declare their mathematical constraint.
 *
 * The scope exposes the local endpoint through [self], the optional neighbor endpoint through
 * [other], the current integration horizon through [timeStep], and the formula slack as an affine
 * term through [slack].  Custom state-dependent values can be lifted into the DSL with [scalar] and
 * [vector].
 *
 * [other] is lazy on purpose: local formulas can ignore it, while pairwise formulas fail early if
 * they are accidentally installed without a neighbor decision vector.
 */
class ControlFunctionScope internal constructor(
    selfDecision: GRBVector,
    otherDecision: GRBVector?,
    val slack: AffineExpression,
) {
    val self: AgentExpression = AgentExpression(selfDecision) { it.self }

    val other: AgentExpression by lazy {
        val decision = checkNotNull(otherDecision) { "Formula requires the neighbor decision vector" }
        AgentExpression(decision) {
            checkNotNull(it.other) { "Formula requires the neighbor device at update time" }
        }
    }

    val timeStep: RuntimeScalar = RuntimeScalar { it.deltaTime }

    /**
     * Lifts a runtime scalar into the formula DSL.
     *
     * Use this for values that are not part of [AgentExpression] but still have to be refreshed at
     * each solver iteration, such as a moving obstacle radius or a target-dependent coefficient.
     */
    fun scalar(block: FormulaRuntime.() -> Double): RuntimeScalar = RuntimeScalar { runtime -> runtime.block() }

    /**
     * Lifts a runtime vector into the formula DSL.
     *
     * The returned vector is evaluated during constraint updates and can be used as a dynamic
     * coefficient vector in expressions such as `dot(distance, self.u)`.
     */
    fun vector(block: FormulaRuntime.() -> DoubleArray): VectorExpression =
        VectorExpression { runtime -> runtime.block() }
}
