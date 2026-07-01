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
) {
    internal companion object {
        fun empty(): AffineExpression = AffineExpression()

        fun constant(value: RuntimeScalar): AffineExpression = AffineExpression(constant = value)

        fun variable(variable: GRBVar, coefficient: RuntimeScalar = scalar(1.0)): AffineExpression =
            AffineExpression(listOf(LinearTerm(variable, coefficient)))
    }
}

/**
 * Adds two affine expressions by concatenating their terms and summing their constants.
 */
operator fun AffineExpression.plus(other: AffineExpression): AffineExpression =
    AffineExpression(terms + other.terms, constant + other.constant)

/**
 * Subtracts one affine expression from another.
 */
operator fun AffineExpression.minus(other: AffineExpression): AffineExpression = this + (-other)

/**
 * Negates all terms and the constant part of this affine expression.
 */
operator fun AffineExpression.unaryMinus(): AffineExpression =
    AffineExpression(terms.map { it.copy(coefficient = -it.coefficient) }, -constant)

/**
 * Multiplies every coefficient and the constant part by a runtime scalar.
 */
operator fun RuntimeScalar.times(expression: AffineExpression): AffineExpression =
    AffineExpression(
        expression.terms.map { it.copy(coefficient = it.coefficient * this) },
        expression.constant * this,
    )

/**
 * Multiplies an affine expression by this constant number.
 */
operator fun Number.times(expression: AffineExpression): AffineExpression = asRuntimeScalar() * expression
