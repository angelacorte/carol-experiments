package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Device
import it.unibo.collektive.solver.gurobi.GRBVector

class AgentExpression internal constructor(
    decision: GRBVector,
    private val device: (FormulaRuntime) -> Device,
) {
    val u: DecisionVector = DecisionVector(decision)

    val position: VectorExpression = VectorExpression { runtime -> device(runtime).position.toDoubleArray() }

    val safeMargin: RuntimeScalar = RuntimeScalar { runtime -> device(runtime).safeMargin }

    val maxSpeed: RuntimeScalar = RuntimeScalar { runtime -> device(runtime).maxSpeed }
}

class ControlFunctionScope internal constructor(
    selfDecision: GRBVector,
    otherDecision: GRBVector?,
    slackVariable: GRBVar?,
) {
    val self: AgentExpression = AgentExpression(selfDecision) { it.self }

    val other: AgentExpression by lazy {
        val decision = checkNotNull(otherDecision) { "Formula requires the neighbor decision vector" }
        AgentExpression(decision) {
            checkNotNull(it.other) { "Formula requires the neighbor device at update time" }
        }
    }

    val timeStep: RuntimeScalar = RuntimeScalar { it.deltaTime }

    val slack: AffineExpression = slackVariable?.let {
        AffineExpression(listOf(LinearTerm(it, scalar(1.0))))
    } ?: AffineExpression()

    /**
     * Lifts a runtime scalar into the formula DSL.
     *
     * Use this for values that are not part of [AgentExpression] but still have to be refreshed at
     * each solver iteration, such as a moving obstacle radius or a target-dependent coefficient.
     */
    fun scalar(block: FormulaRuntime.() -> Double): RuntimeScalar =
        RuntimeScalar { runtime -> runtime.block() }

    fun vector(block: FormulaRuntime.() -> DoubleArray): VectorExpression =
        VectorExpression { runtime -> runtime.block() }
}
