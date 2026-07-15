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
    /** Adds two affine expressions by concatenating their terms and summing their constants. */
    operator fun plus(other: AffineExpression): AffineExpression =
        AffineExpression(terms + other.terms, constant + other.constant)

    /** Subtracts one affine expression from another. */
    operator fun minus(other: AffineExpression): AffineExpression = this + (-other)

    /** Negates all terms and the constant part of this affine expression. */
    operator fun unaryMinus(): AffineExpression =
        AffineExpression(terms.map { it.copy(coefficient = -it.coefficient) }, -constant)

    internal companion object {
        fun empty(): AffineExpression = AffineExpression()

        fun variable(variable: GRBVar, coefficient: RuntimeScalar = scalar(1.0)): AffineExpression =
            AffineExpression(listOf(LinearTerm(variable, coefficient)))
    }
}
