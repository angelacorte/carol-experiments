package it.unibo.collektive.control.dsl.expressions

import com.gurobi.gurobi.GRBVar

/**
 * One quadratic term attached to two concrete Gurobi variables.
 */
internal data class QuadraticTerm(val coefficient: Double, val first: GRBVar, val second: GRBVar)

/**
 * Quadratic expression with a fixed Gurobi variable structure.
 *
 * This type intentionally stores only static quadratic coefficients.  Runtime changes are currently
 * supported on the constraint RHS, matching the reusable-model protocol used by the solver.
 */
class QuadraticExpression internal constructor(internal val terms: List<QuadraticTerm>)

/**
 * Squared Euclidean norm of a decision vector.
 *
 * The returned expression has a fixed quadratic structure and can therefore be installed once as a
 * Gurobi quadratic constraint.
 */
fun squaredNorm(decision: DecisionVector): QuadraticExpression = QuadraticExpression(
    List(decision.dimensions) { index ->
        val variable = decision.vector[index]
        QuadraticTerm(1.0, variable, variable)
    },
)

/**
 * Scales every quadratic coefficient by a constant number.
 */
operator fun QuadraticExpression.times(factor: Number): QuadraticExpression =
    QuadraticExpression(terms.map { it.copy(coefficient = it.coefficient * factor.toDouble()) })

/**
 * Scales a quadratic expression by this constant number.
 */
operator fun Number.times(expression: QuadraticExpression): QuadraticExpression = expression * this
