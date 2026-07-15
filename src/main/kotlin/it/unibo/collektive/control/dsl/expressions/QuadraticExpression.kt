package it.unibo.collektive.control.dsl.expressions

import com.gurobi.gurobi.GRBVar

/** One quadratic term attached to two concrete Gurobi variables. */
internal data class QuadraticTerm(val coefficient: Double, val first: GRBVar, val second: GRBVar)

/**
 * Quadratic expression with a fixed Gurobi variable structure.
 *
 * This type intentionally stores only static quadratic coefficients: runtime changes are currently
 * supported on the constraint RHS only, matching the reusable-model protocol used by the solver.
 */
class QuadraticExpression internal constructor(internal val terms: List<QuadraticTerm>)
