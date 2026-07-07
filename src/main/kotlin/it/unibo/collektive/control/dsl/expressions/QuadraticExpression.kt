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

/**
 * Squared Euclidean norm of a decision expression.
 *
 * Works for a raw decision vector (`self.u`) as well as for any static linear combination of
 * decision vectors (e.g. `self.u - other.u`), because [DecisionExpression] always carries the underlying
 * Gurobi variables together with the constant coefficient contributed by each one. Each dimension
 * `sum_k c_k * x_k` is expanded into its full quadratic form `sum_{k,l} c_k * c_l * x_k * x_l`, which
 * correctly reduces to the previous single-variable case when there is exactly one term per
 * dimension.
 */
fun squaredNorm(decision: DecisionExpression): QuadraticExpression = QuadraticExpression(
    decision.components.flatMap { component ->
        component.indices.flatMap { i ->
            component.indices.map { j ->
                val (variableI, coefficientI) = component[i]
                val (variableJ, coefficientJ) = component[j]
                QuadraticTerm(coefficientI * coefficientJ, variableI, variableJ)
            }
        }
    },
)
