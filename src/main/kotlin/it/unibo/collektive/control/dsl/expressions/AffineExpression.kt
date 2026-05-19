package it.unibo.collektive.control.dsl.expressions

import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.dsl.FormulaRuntime

/**
 * One affine term attached to a concrete Gurobi variable.
 */
internal data class LinearTerm(val variable: GRBVar, val coefficient: RuntimeScalar)

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

/**
 * Adds two affine expressions by concatenating their terms and summing their constants.
 */
operator fun AffineExpression.plus(other: AffineExpression): AffineExpression =
    AffineExpression(terms + other.terms, constant + other.constant)

/**
 * Adds a runtime scalar to the constant part of this affine expression.
 */
operator fun AffineExpression.plus(other: RuntimeScalar): AffineExpression = this + affine(other)

/**
 * Adds a constant number to the constant part of this affine expression.
 */
operator fun AffineExpression.plus(other: Number): AffineExpression = this + affine(other.toDouble())

/**
 * Adds an affine expression to this runtime scalar.
 */
operator fun RuntimeScalar.plus(other: AffineExpression): AffineExpression = affine(this) + other

/**
 * Adds an affine expression to this constant number.
 */
operator fun Number.plus(other: AffineExpression): AffineExpression = affine(toDouble()) + other

/**
 * Subtracts one affine expression from another.
 */
operator fun AffineExpression.minus(other: AffineExpression): AffineExpression = this + (-other)

/**
 * Subtracts a runtime scalar from the constant part of this affine expression.
 */
operator fun AffineExpression.minus(other: RuntimeScalar): AffineExpression = this - affine(other)

/**
 * Subtracts a constant number from the constant part of this affine expression.
 */
operator fun AffineExpression.minus(other: Number): AffineExpression = this - affine(other.toDouble())

/**
 * Subtracts an affine expression from this runtime scalar.
 */
operator fun RuntimeScalar.minus(other: AffineExpression): AffineExpression = affine(this) - other

/**
 * Subtracts an affine expression from this constant number.
 */
operator fun Number.minus(other: AffineExpression): AffineExpression = affine(toDouble()) - other

/**
 * Negates all terms and the constant part of this affine expression.
 */
operator fun AffineExpression.unaryMinus(): AffineExpression =
    AffineExpression(terms.map { it.copy(coefficient = -it.coefficient) }, -constant)

/**
 * Multiplies every coefficient and the constant part by a runtime scalar.
 */
operator fun AffineExpression.times(factor: RuntimeScalar): AffineExpression =
    AffineExpression(terms.map { it.copy(coefficient = it.coefficient * factor) }, constant * factor)

/**
 * Multiplies every coefficient and the constant part by a constant number.
 */
operator fun AffineExpression.times(factor: Number): AffineExpression = this * scalar(factor.toDouble())

/**
 * Multiplies an affine expression by this runtime scalar.
 */
operator fun RuntimeScalar.times(expression: AffineExpression): AffineExpression = expression * this

/**
 * Multiplies an affine expression by this constant number.
 */
operator fun Number.times(expression: AffineExpression): AffineExpression = expression * toDouble()
