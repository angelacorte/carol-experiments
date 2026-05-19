package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.solver.gurobi.GRBVector
import kotlin.math.max

/**
 * Runtime scalar expression.  It can depend on the current devices, settings, and time step.
 */
class ScalarExpression internal constructor(
    internal val evaluate: (FormulaRuntime) -> Double,
)

fun scalar(value: Double): ScalarExpression = ScalarExpression { value }

operator fun ScalarExpression.plus(other: ScalarExpression): ScalarExpression =
    ScalarExpression { evaluate(it) + other.evaluate(it) }

operator fun ScalarExpression.plus(other: Double): ScalarExpression = this + scalar(other)

operator fun Double.plus(other: ScalarExpression): ScalarExpression = scalar(this) + other

operator fun Int.plus(other: ScalarExpression): ScalarExpression = toDouble() + other

operator fun ScalarExpression.minus(other: ScalarExpression): ScalarExpression =
    ScalarExpression { evaluate(it) - other.evaluate(it) }

operator fun ScalarExpression.minus(other: Double): ScalarExpression = this - scalar(other)

operator fun Double.minus(other: ScalarExpression): ScalarExpression = scalar(this) - other

operator fun Int.minus(other: ScalarExpression): ScalarExpression = toDouble() - other

operator fun ScalarExpression.unaryMinus(): ScalarExpression = ScalarExpression { -evaluate(it) }

operator fun ScalarExpression.times(other: ScalarExpression): ScalarExpression =
    ScalarExpression { evaluate(it) * other.evaluate(it) }

operator fun ScalarExpression.times(other: Double): ScalarExpression = this * scalar(other)

operator fun Double.times(other: ScalarExpression): ScalarExpression = scalar(this) * other

operator fun Int.times(other: ScalarExpression): ScalarExpression = toDouble() * other

operator fun ScalarExpression.div(other: ScalarExpression): ScalarExpression =
    ScalarExpression { evaluate(it) / other.evaluate(it) }

operator fun ScalarExpression.div(other: Double): ScalarExpression = this / scalar(other)

operator fun Double.div(other: ScalarExpression): ScalarExpression = scalar(this) / other

operator fun Int.div(other: ScalarExpression): ScalarExpression = toDouble() / other

fun squared(value: Double): ScalarExpression = scalar(value * value)

fun squared(value: ScalarExpression): ScalarExpression = value * value

fun max(left: ScalarExpression, right: ScalarExpression): ScalarExpression =
    ScalarExpression { max(left.evaluate(it), right.evaluate(it)) }

/**
 * Runtime vector expression.  It is intended for state-dependent coefficients, not decision variables.
 */
class VectorExpression internal constructor(
    internal val evaluate: (FormulaRuntime) -> DoubleArray,
)

operator fun VectorExpression.plus(other: VectorExpression): VectorExpression =
    VectorExpression { runtime ->
        val left = evaluate(runtime)
        val right = other.evaluate(runtime)
        require(left.size == right.size) { "Vector dimension mismatch: ${left.size} != ${right.size}" }
        DoubleArray(left.size) { i -> left[i] + right[i] }
    }

operator fun VectorExpression.minus(other: VectorExpression): VectorExpression =
    VectorExpression { runtime ->
        val left = evaluate(runtime)
        val right = other.evaluate(runtime)
        require(left.size == right.size) { "Vector dimension mismatch: ${left.size} != ${right.size}" }
        DoubleArray(left.size) { i -> left[i] - right[i] }
    }

operator fun VectorExpression.unaryMinus(): VectorExpression =
    VectorExpression { runtime -> evaluate(runtime).map { -it }.toDoubleArray() }

operator fun ScalarExpression.times(vector: VectorExpression): VectorExpression =
    VectorExpression { runtime ->
        val factor = evaluate(runtime)
        vector.evaluate(runtime).map { factor * it }.toDoubleArray()
    }

operator fun Double.times(vector: VectorExpression): VectorExpression = scalar(this) * vector

operator fun Int.times(vector: VectorExpression): VectorExpression = toDouble() * vector

fun squaredNorm(vector: VectorExpression): ScalarExpression =
    ScalarExpression { runtime -> vector.evaluate(runtime).sumOf { it * it } }

class DecisionVector internal constructor(
    internal val vector: GRBVector,
) {
    val dimensions: Int get() = vector.dimensions

    internal fun asExpression(): DecisionVectorExpression =
        DecisionVectorExpression(listOf(DecisionVectorTerm(scalar(1.0), this)))
}

internal data class DecisionVectorTerm(
    val coefficient: ScalarExpression,
    val vector: DecisionVector,
)

class DecisionVectorExpression internal constructor(
    internal val terms: List<DecisionVectorTerm>,
)

operator fun DecisionVector.plus(other: DecisionVector): DecisionVectorExpression = asExpression() + other.asExpression()

operator fun DecisionVector.minus(other: DecisionVector): DecisionVectorExpression = asExpression() - other.asExpression()

operator fun DecisionVector.unaryMinus(): DecisionVectorExpression = -asExpression()

operator fun DecisionVectorExpression.plus(other: DecisionVectorExpression): DecisionVectorExpression =
    DecisionVectorExpression(terms + other.terms)

operator fun DecisionVectorExpression.plus(other: DecisionVector): DecisionVectorExpression = this + other.asExpression()

operator fun DecisionVectorExpression.minus(other: DecisionVectorExpression): DecisionVectorExpression =
    this + (-other)

operator fun DecisionVectorExpression.minus(other: DecisionVector): DecisionVectorExpression = this - other.asExpression()

operator fun DecisionVectorExpression.unaryMinus(): DecisionVectorExpression =
    DecisionVectorExpression(terms.map { DecisionVectorTerm(-it.coefficient, it.vector) })

operator fun ScalarExpression.times(decision: DecisionVectorExpression): DecisionVectorExpression =
    DecisionVectorExpression(decision.terms.map { it.copy(coefficient = it.coefficient * this) })

operator fun Double.times(decision: DecisionVectorExpression): DecisionVectorExpression = scalar(this) * decision

operator fun Int.times(decision: DecisionVectorExpression): DecisionVectorExpression = toDouble() * decision

operator fun ScalarExpression.times(decision: DecisionVector): DecisionVectorExpression = this * decision.asExpression()

operator fun Double.times(decision: DecisionVector): DecisionVectorExpression = scalar(this) * decision

operator fun Int.times(decision: DecisionVector): DecisionVectorExpression = toDouble() * decision

internal data class LinearTerm(
    val variable: GRBVar,
    val coefficient: ScalarExpression,
)

class AffineExpression internal constructor(
    internal val terms: List<LinearTerm> = emptyList(),
    internal val constant: ScalarExpression = scalar(0.0),
)

fun affine(value: Double): AffineExpression = AffineExpression(constant = scalar(value))

fun affine(value: ScalarExpression): AffineExpression = AffineExpression(constant = value)

operator fun AffineExpression.plus(other: AffineExpression): AffineExpression =
    AffineExpression(terms + other.terms, constant + other.constant)

operator fun AffineExpression.plus(other: ScalarExpression): AffineExpression = this + affine(other)

operator fun AffineExpression.plus(other: Double): AffineExpression = this + affine(other)

operator fun ScalarExpression.plus(other: AffineExpression): AffineExpression = affine(this) + other

operator fun Double.plus(other: AffineExpression): AffineExpression = affine(this) + other

operator fun Int.plus(other: AffineExpression): AffineExpression = toDouble() + other

operator fun AffineExpression.minus(other: AffineExpression): AffineExpression = this + (-other)

operator fun AffineExpression.minus(other: ScalarExpression): AffineExpression = this - affine(other)

operator fun AffineExpression.minus(other: Double): AffineExpression = this - affine(other)

operator fun ScalarExpression.minus(other: AffineExpression): AffineExpression = affine(this) - other

operator fun Double.minus(other: AffineExpression): AffineExpression = affine(this) - other

operator fun Int.minus(other: AffineExpression): AffineExpression = toDouble() - other

operator fun AffineExpression.unaryMinus(): AffineExpression =
    AffineExpression(terms.map { it.copy(coefficient = -it.coefficient) }, -constant)

operator fun AffineExpression.times(factor: ScalarExpression): AffineExpression =
    AffineExpression(terms.map { it.copy(coefficient = it.coefficient * factor) }, constant * factor)

operator fun AffineExpression.times(factor: Double): AffineExpression = this * scalar(factor)

operator fun ScalarExpression.times(expression: AffineExpression): AffineExpression = expression * this

operator fun Double.times(expression: AffineExpression): AffineExpression = expression * this

operator fun Int.times(expression: AffineExpression): AffineExpression = toDouble() * expression

fun dot(coefficients: VectorExpression, decision: DecisionVector): AffineExpression =
    dot(coefficients, decision.asExpression())

fun dot(coefficients: VectorExpression, decision: DecisionVectorExpression): AffineExpression {
    val linearTerms = decision.terms.flatMap { decisionTerm ->
        val dimension = decisionTerm.vector.dimensions
        List(dimension) { index ->
            LinearTerm(
                variable = decisionTerm.vector.vector[index],
                coefficient = ScalarExpression { runtime ->
                    val values = coefficients.evaluate(runtime)
                    require(values.size == dimension) {
                        "Coefficient dimension mismatch: ${values.size} != $dimension"
                    }
                    values[index] * decisionTerm.coefficient.evaluate(runtime)
                },
            )
        }
    }
    return AffineExpression(linearTerms)
}

internal data class QuadraticTerm(
    val coefficient: Double,
    val first: GRBVar,
    val second: GRBVar,
)

class QuadraticExpression internal constructor(
    internal val terms: List<QuadraticTerm>,
)

fun squaredNorm(decision: DecisionVector): QuadraticExpression =
    QuadraticExpression(
        List(decision.dimensions) { index ->
            val variable = decision.vector[index]
            QuadraticTerm(1.0, variable, variable)
        },
    )

operator fun QuadraticExpression.times(factor: Double): QuadraticExpression =
    QuadraticExpression(terms.map { it.copy(coefficient = it.coefficient * factor) })

operator fun Double.times(expression: QuadraticExpression): QuadraticExpression = expression * this

operator fun Int.times(expression: QuadraticExpression): QuadraticExpression = expression * toDouble()
