package it.unibo.collektive.control.dsl.expressions

import it.unibo.collektive.control.dsl.FormulaRuntime
import kotlin.math.max

/**
 * Runtime expression evaluated against the current formula update context.
 */
internal fun interface RuntimeExpression<out T> {
    fun evaluate(runtime: FormulaRuntime): T
}

/**
 * Scalar value that is evaluated against the current [FormulaRuntime].
 *
 * This type is the scalar building block of the DSL.  It represents both constant numbers and
 * values that change at every solver iteration, such as `timeStep`, `self.maxSpeed`, or an obstacle
 * radius.  Formula compilation stores these expressions and evaluates them during
 * [it.unibo.collektive.solver.gurobi.InstalledControlConstraint.update].
 */
class RuntimeScalar internal constructor(private val expression: RuntimeExpression<Double>) {
    internal fun evaluate(runtime: FormulaRuntime): Double = expression.evaluate(runtime)
}

/**
 * Creates a constant runtime scalar.
 *
 * Constants are represented with the same type as dynamic values so formulas can freely combine
 * literal coefficients and state-dependent quantities.
 */
fun scalar(value: Double): RuntimeScalar = RuntimeScalar { value }

/**
 * Converts any numeric literal into the scalar expression domain.
 */
internal fun Number.asRuntimeScalar(): RuntimeScalar = scalar(toDouble())

/**
 * Squares a constant number and lifts it into the runtime scalar domain.
 */
fun squared(value: Number): RuntimeScalar = scalar(value.toDouble() * value.toDouble())

/**
 * Squares a runtime scalar without evaluating it immediately.
 */
fun squared(value: RuntimeScalar): RuntimeScalar = value * value

/**
 * Runtime maximum between two scalar expressions.
 */
fun max(left: RuntimeScalar, right: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { max(left.evaluate(it), right.evaluate(it)) }

/**
 * Divides a constant number by a runtime scalar.
 */
operator fun Number.div(other: RuntimeScalar): RuntimeScalar = asRuntimeScalar() / other

/**
 * Adds two runtime scalars.
 */
operator fun RuntimeScalar.plus(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) + other.evaluate(it) }

/**
 * Subtracts one runtime scalar from another.
 */
operator fun RuntimeScalar.minus(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) - other.evaluate(it) }

/**
 * Negates this runtime scalar.
 */
operator fun RuntimeScalar.unaryMinus(): RuntimeScalar = RuntimeScalar { -evaluate(it) }

/**
 * Multiplies two runtime scalars.
 */
operator fun RuntimeScalar.times(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) * other.evaluate(it) }

/**
 * Multiplies a runtime scalar by this constant number.
 */
operator fun Number.times(other: RuntimeScalar): RuntimeScalar = asRuntimeScalar() * other

/**
 * Divides this runtime scalar by another runtime scalar.
 */
operator fun RuntimeScalar.div(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) / other.evaluate(it) }
