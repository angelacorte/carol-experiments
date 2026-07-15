package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRB
import it.unibo.collektive.control.dsl.expressions.AffineExpression
import it.unibo.collektive.control.dsl.expressions.DecisionExpression
import it.unibo.collektive.control.dsl.expressions.LinearTerm
import it.unibo.collektive.control.dsl.expressions.QuadraticExpression
import it.unibo.collektive.control.dsl.expressions.QuadraticTerm
import it.unibo.collektive.control.dsl.expressions.RuntimeScalar
import it.unibo.collektive.control.dsl.expressions.VectorExpression
import it.unibo.collektive.control.dsl.expressions.asDecisionExpr
import it.unibo.collektive.control.dsl.expressions.asRuntimeScalar
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
 * @property u decision expression associated with this endpoint.
 * @property position current endpoint position as a runtime vector.
 * @property safeMargin current safety radius contribution.
 * @property maxSpeed current speed bound.
 */
class AgentExpression internal constructor(decision: GRBVector, private val device: (FormulaRuntime) -> Device) {
    val u: DecisionExpression = decision.asDecisionExpr()

    val position: VectorExpression = vector { it.position.toDoubleArray() }

    val safeMargin: RuntimeScalar = scalar(Device::safeMargin)

    val maxSpeed: RuntimeScalar = scalar(Device::maxSpeed)

    private fun scalar(read: (Device) -> Double): RuntimeScalar = RuntimeScalar { runtime -> read(device(runtime)) }

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
 * The scope is also the single entry point for the formula vocabulary: numeric multipliers
 * (`2.0 * ...`), [dot], [squaredNorm], [squared], [max], and the infix constraint builders
 * [lessThanOrEqualTo] / [greaterThanOrEqualTo] are all members, so formulas need no DSL imports.
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

    /** Multiplies a runtime scalar by this constant number. */
    operator fun Number.times(other: RuntimeScalar): RuntimeScalar = asRuntimeScalar() * other

    /** Multiplies an affine expression by this constant number. */
    operator fun Number.times(affine: AffineExpression): AffineExpression = asRuntimeScalar() * affine

    /** Divides a constant number by a runtime scalar. */
    operator fun Number.div(other: RuntimeScalar): RuntimeScalar = asRuntimeScalar() / other

    /** Squares a constant number and lifts it into the runtime scalar domain. */
    fun squared(value: Number): RuntimeScalar = (value.toDouble() * value.toDouble()).asRuntimeScalar()

    /** Squares a runtime scalar without evaluating it immediately. */
    fun squared(value: RuntimeScalar): RuntimeScalar = value * value

    /** Runtime maximum between two scalar expressions. */
    fun max(left: RuntimeScalar, right: RuntimeScalar): RuntimeScalar =
        RuntimeScalar { kotlin.math.max(left.evaluate(it), right.evaluate(it)) }

    /**
     * Builds an affine dot product between runtime coefficients and a decision expression.
     *
     * [decision] may be a raw decision vector (`self.u`) or any static combination of decision
     * vectors (`self.u - other.u`).  The resulting expression is affine in Gurobi variables, while
     * [coefficients] can still depend on the current runtime state.
     */
    fun dot(coefficients: VectorExpression, decision: DecisionExpression): AffineExpression = AffineExpression(
        decision.components.flatMapIndexed { index, componentTerms ->
            componentTerms.map { (variable, coefficient) ->
                LinearTerm(
                    variable,
                    RuntimeScalar { runtime ->
                        val values = coefficients.evaluate(runtime)
                        require(values.size == decision.dimensions) {
                            "Coefficient dimension mismatch: ${values.size} != ${decision.dimensions}"
                        }
                        values[index] * coefficient
                    },
                )
            }
        },
    )

    /** Squared Euclidean norm of a runtime vector. */
    fun squaredNorm(vector: VectorExpression): RuntimeScalar =
        RuntimeScalar { runtime -> vector.evaluate(runtime).sumOf { it * it } }

    /**
     * Squared Euclidean norm of a decision expression.
     *
     * Works for a raw decision vector (`self.u`) as well as for any static linear combination of
     * decision vectors (e.g. `self.u - other.u`): each dimension `sum_k c_k * x_k` is expanded into
     * its full quadratic form `sum_{k,l} c_k * c_l * x_k * x_l`.
     */
    fun squaredNorm(decision: DecisionExpression): QuadraticExpression = QuadraticExpression(
        decision.components.flatMap { component ->
            component.flatMap { (variableI, coefficientI) ->
                component.map { (variableJ, coefficientJ) ->
                    QuadraticTerm(coefficientI * coefficientJ, variableI, variableJ)
                }
            }
        },
    )

    /** Builds a linear formula `this <= rightHandSide`. */
    infix fun AffineExpression.lessThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
        LinearConstraintFormula(this, GRB.LESS_EQUAL, rightHandSide)

    /** Builds a linear formula `this >= rightHandSide`. */
    infix fun AffineExpression.greaterThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
        LinearConstraintFormula(this, GRB.GREATER_EQUAL, rightHandSide)

    /** Builds a quadratic formula `this <= rightHandSide`. */
    infix fun QuadraticExpression.lessThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
        QuadraticConstraintFormula(this, GRB.LESS_EQUAL, rightHandSide)
}
