package it.unibo.collektive.control.dsl.expressions

import it.unibo.collektive.control.dsl.FormulaRuntime
import kotlin.math.max

/**
 * Scalar value that is evaluated against the current [FormulaRuntime].
 *
 * This type is the scalar building block of the DSL.  It represents both constant numbers and
 * values that change at every solver iteration, such as `timeStep`, `self.maxSpeed`, or an obstacle
 * radius.  Formula compilation stores these expressions and evaluates them during
 * [it.unibo.collektive.solver.gurobi.Constraint.update].
 */
class RuntimeScalar internal constructor(
    internal val evaluate: (FormulaRuntime) -> Double,
)

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

operator fun Number.div(other: RuntimeScalar): RuntimeScalar = asRuntimeScalar() / other
operator fun RuntimeScalar.plus(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) + other.evaluate(it) }

operator fun RuntimeScalar.plus(other: Number): RuntimeScalar = this + other.asRuntimeScalar()

operator fun Number.plus(other: RuntimeScalar): RuntimeScalar = asRuntimeScalar() + other

operator fun RuntimeScalar.minus(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) - other.evaluate(it) }

operator fun RuntimeScalar.minus(other: Number): RuntimeScalar = this - other.asRuntimeScalar()

operator fun Number.minus(other: RuntimeScalar): RuntimeScalar = asRuntimeScalar() - other

operator fun RuntimeScalar.unaryMinus(): RuntimeScalar = RuntimeScalar { -evaluate(it) }

operator fun RuntimeScalar.times(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) * other.evaluate(it) }

operator fun RuntimeScalar.times(other: Number): RuntimeScalar = this * other.asRuntimeScalar()

operator fun Number.times(other: RuntimeScalar): RuntimeScalar = asRuntimeScalar() * other

operator fun RuntimeScalar.div(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) / other.evaluate(it) }

operator fun RuntimeScalar.div(other: Number): RuntimeScalar = this / other.asRuntimeScalar()
