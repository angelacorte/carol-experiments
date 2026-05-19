package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.solver.gurobi.GRBVector
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
private fun Number.asScalarExpression(): RuntimeScalar = scalar(toDouble())

operator fun RuntimeScalar.plus(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) + other.evaluate(it) }

operator fun RuntimeScalar.plus(other: Number): RuntimeScalar = this + other.asScalarExpression()

operator fun Number.plus(other: RuntimeScalar): RuntimeScalar = asScalarExpression() + other

operator fun RuntimeScalar.minus(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) - other.evaluate(it) }

operator fun RuntimeScalar.minus(other: Number): RuntimeScalar = this - other.asScalarExpression()

operator fun Number.minus(other: RuntimeScalar): RuntimeScalar = asScalarExpression() - other

operator fun RuntimeScalar.unaryMinus(): RuntimeScalar = RuntimeScalar { -evaluate(it) }

operator fun RuntimeScalar.times(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) * other.evaluate(it) }

operator fun RuntimeScalar.times(other: Number): RuntimeScalar = this * other.asScalarExpression()

operator fun Number.times(other: RuntimeScalar): RuntimeScalar = asScalarExpression() * other

operator fun RuntimeScalar.div(other: RuntimeScalar): RuntimeScalar =
    RuntimeScalar { evaluate(it) / other.evaluate(it) }

operator fun RuntimeScalar.div(other: Number): RuntimeScalar = this / other.asScalarExpression()

operator fun Number.div(other: RuntimeScalar): RuntimeScalar = asScalarExpression() / other

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
 * Runtime vector expression used for state-dependent coefficients.
 *
 * Unlike [DecisionVector], this does not represent Gurobi variables.  It is evaluated during
 * constraint updates and is typically used for quantities such as relative positions or target
 * offsets that become coefficients in affine expressions.
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

operator fun RuntimeScalar.times(vector: VectorExpression): VectorExpression =
    VectorExpression { runtime ->
        val factor = evaluate(runtime)
        vector.evaluate(runtime).map { factor * it }.toDoubleArray()
    }

operator fun Number.times(vector: VectorExpression): VectorExpression = asScalarExpression() * vector

/**
 * Squared Euclidean norm of a runtime vector.
 */
fun squaredNorm(vector: VectorExpression): RuntimeScalar =
    RuntimeScalar { runtime -> vector.evaluate(runtime).sumOf { it * it } }

/**
 * Vector of Gurobi decision variables exposed to formulas.
 *
 * A local QP exposes its control variable as `self.u`; a pairwise QP exposes both `self.u` and
 * `other.u`.  The type is kept separate from [VectorExpression] so the DSL can distinguish dynamic
 * numeric coefficients from optimization variables.
 */
class DecisionVector internal constructor(
    internal val vector: GRBVector,
) {
    val dimensions: Int get() = vector.dimensions

    internal fun asExpression(): DecisionVectorExpression =
        DecisionVectorExpression(listOf(DecisionVectorTerm(scalar(1.0), this)))
}

/**
 * One scaled decision vector inside a [DecisionVectorExpression].
 */
internal data class DecisionVectorTerm(
    val coefficient: RuntimeScalar,
    val vector: DecisionVector,
)

/**
 * Linear combination of decision vectors.
 *
 * This is used for expressions such as `self.u - other.u`.  It is not a Gurobi expression yet; it is
 * an intermediate representation that becomes concrete linear terms once paired with coefficients
 * through [dot].
 */
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

operator fun RuntimeScalar.times(decision: DecisionVectorExpression): DecisionVectorExpression =
    DecisionVectorExpression(decision.terms.map { it.copy(coefficient = it.coefficient * this) })

operator fun Number.times(decision: DecisionVectorExpression): DecisionVectorExpression = asScalarExpression() * decision

operator fun RuntimeScalar.times(decision: DecisionVector): DecisionVectorExpression = this * decision.asExpression()

operator fun Number.times(decision: DecisionVector): DecisionVectorExpression = asScalarExpression() * decision

/**
 * One affine term attached to a concrete Gurobi variable.
 */
internal data class LinearTerm(
    val variable: GRBVar,
    val coefficient: RuntimeScalar,
)

/**
 * Affine expression in Gurobi decision variables with runtime coefficients.
 *
 * The expression stores linear terms and a runtime constant separately.  During installation the
 * variable structure is added once with placeholder coefficients; during updates the coefficients
 * and the shifted RHS are refreshed from [FormulaRuntime].
 */
class AffineExpression internal constructor(
    internal val terms: List<LinearTerm> = emptyList(),
    internal val constant: RuntimeScalar = scalar(0.0),
)

/**
 * Creates a constant affine expression.
 */
fun affine(value: Double): AffineExpression = AffineExpression(constant = scalar(value))

/**
 * Creates an affine expression from a runtime scalar.
 */
fun affine(value: RuntimeScalar): AffineExpression = AffineExpression(constant = value)

operator fun AffineExpression.plus(other: AffineExpression): AffineExpression =
    AffineExpression(terms + other.terms, constant + other.constant)

operator fun AffineExpression.plus(other: RuntimeScalar): AffineExpression = this + affine(other)

operator fun AffineExpression.plus(other: Number): AffineExpression = this + affine(other.toDouble())

operator fun RuntimeScalar.plus(other: AffineExpression): AffineExpression = affine(this) + other

operator fun Number.plus(other: AffineExpression): AffineExpression = affine(toDouble()) + other

operator fun AffineExpression.minus(other: AffineExpression): AffineExpression = this + (-other)

operator fun AffineExpression.minus(other: RuntimeScalar): AffineExpression = this - affine(other)

operator fun AffineExpression.minus(other: Number): AffineExpression = this - affine(other.toDouble())

operator fun RuntimeScalar.minus(other: AffineExpression): AffineExpression = affine(this) - other

operator fun Number.minus(other: AffineExpression): AffineExpression = affine(toDouble()) - other

operator fun AffineExpression.unaryMinus(): AffineExpression =
    AffineExpression(terms.map { it.copy(coefficient = -it.coefficient) }, -constant)

operator fun AffineExpression.times(factor: RuntimeScalar): AffineExpression =
    AffineExpression(terms.map { it.copy(coefficient = it.coefficient * factor) }, constant * factor)

operator fun AffineExpression.times(factor: Number): AffineExpression = this * scalar(factor.toDouble())

operator fun RuntimeScalar.times(expression: AffineExpression): AffineExpression = expression * this

operator fun Number.times(expression: AffineExpression): AffineExpression = expression * toDouble()

/**
 * Builds an affine dot product between runtime coefficients and one decision vector.
 */
fun dot(coefficients: VectorExpression, decision: DecisionVector): AffineExpression =
    dot(coefficients, decision.asExpression())

/**
 * Builds an affine dot product between runtime coefficients and a linear combination of decisions.
 *
 * The resulting expression is affine in Gurobi variables, while the coefficients can still depend on
 * the current runtime state.
 */
fun dot(coefficients: VectorExpression, decision: DecisionVectorExpression): AffineExpression {
    val linearTerms = decision.terms.flatMap { decisionTerm ->
        val dimension = decisionTerm.vector.dimensions
        List(dimension) { index ->
            LinearTerm(
                variable = decisionTerm.vector.vector[index],
                coefficient = RuntimeScalar { runtime ->
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

/**
 * One quadratic term attached to two concrete Gurobi variables.
 */
internal data class QuadraticTerm(
    val coefficient: Double,
    val first: GRBVar,
    val second: GRBVar,
)

/**
 * Quadratic expression with a fixed Gurobi variable structure.
 *
 * This type intentionally stores only static quadratic coefficients.  Runtime changes are currently
 * supported on the constraint RHS, matching the reusable-model protocol used by the solver.
 */
class QuadraticExpression internal constructor(
    internal val terms: List<QuadraticTerm>,
)

/**
 * Squared Euclidean norm of a decision vector.
 *
 * The returned expression has a fixed quadratic structure and can therefore be installed once as a
 * Gurobi quadratic constraint.
 */
fun squaredNorm(decision: DecisionVector): QuadraticExpression =
    QuadraticExpression(
        List(decision.dimensions) { index ->
            val variable = decision.vector[index]
            QuadraticTerm(1.0, variable, variable)
        },
    )

operator fun QuadraticExpression.times(factor: Number): QuadraticExpression =
    QuadraticExpression(terms.map { it.copy(coefficient = it.coefficient * factor.toDouble()) })

operator fun Number.times(expression: QuadraticExpression): QuadraticExpression = expression * this
