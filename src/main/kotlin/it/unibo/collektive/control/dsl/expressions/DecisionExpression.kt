package it.unibo.collektive.control.dsl.expressions

import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.solver.gurobi.GRBVector

/** One static (installation-time constant) coefficient attached to a Gurobi decision variable. */
internal data class StaticTerm(val variable: GRBVar, val coefficient: Double)

/**
 * Linear combination of Gurobi decision variables with static (installation-time) coefficients.
 *
 * `self.u` is a plain [DecisionExpression] where each dimension carries a single term with coefficient
 * `1.0`. Combining decision vectors — e.g. `self.u - other.u` — produces a [DecisionExpression] whose
 * dimensions carry more than one term. Coefficients here are always static: they only ever arise
 * from combining decision vectors with `+`, `-`, or unary `-`. Any *dynamic* scaling of a decision
 * vector must instead go through [dot], which multiplies by a runtime-evaluated [VectorExpression].
 *
 * This type unifies what used to be two separate types (a raw Gurobi vector, and a combination of
 * such vectors): both [dot] and `squaredNorm` accept a single [DecisionExpression], so
 * `squaredNorm(self.u - other.u)` works exactly like `squaredNorm(self.u)`.
 */
class DecisionExpression internal constructor(internal val components: List<List<StaticTerm>>) {
    /** Number of scalar dimensions represented by this decision expression. */
    val dimensions: Int get() = components.size
}

/**
 * Wraps a raw Gurobi decision vector as a [DecisionExpression] with one unit-coefficient term per dimension.
 */
internal fun GRBVector.asDecisionExpr(): DecisionExpression = DecisionExpression(
    List(dimensions) { index -> listOf(StaticTerm(this[index], 1.0)) },
)

/** Adds two decision expressions dimension by dimension. */
operator fun DecisionExpression.plus(other: DecisionExpression): DecisionExpression = combine(other) { left, right ->
    left +
        right
}

/** Subtracts one decision expression from another dimension by dimension. */
operator fun DecisionExpression.minus(other: DecisionExpression): DecisionExpression =
    combine(other) { left, right -> left + right.map { it.copy(coefficient = -it.coefficient) } }

/** Negates every term of this decision expression. */
operator fun DecisionExpression.unaryMinus(): DecisionExpression =
    DecisionExpression(components.map { component -> component.map { it.copy(coefficient = -it.coefficient) } })

private fun DecisionExpression.combine(
    other: DecisionExpression,
    component: (List<StaticTerm>, List<StaticTerm>) -> List<StaticTerm>,
): DecisionExpression {
    require(dimensions == other.dimensions) {
        "Decision expression dimension mismatch: $dimensions != ${other.dimensions}"
    }
    return DecisionExpression(components.zip(other.components).map { (left, right) -> component(left, right) })
}
