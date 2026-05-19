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
