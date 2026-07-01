package it.unibo.collektive.control.dsl.expressions

import it.unibo.collektive.solver.gurobi.GRBVector

/**
 * Vector of Gurobi decision variables exposed to formulas.
 *
 * A local QP exposes its control variable as `self.u`; a pairwise QP exposes both `self.u` and
 * `other.u`.  The type is kept separate from [VectorExpression] so the DSL can distinguish dynamic
 * numeric coefficients from optimization variables.
 */
class DecisionVector internal constructor(internal val vector: GRBVector) {
    /** Number of scalar Gurobi variables in this decision vector. */
    val dimensions: Int get() = vector.dimensions

    internal fun asExpression(): DecisionVectorExpression = DecisionVectorExpression(
        dimensions = dimensions,
        components = List(dimensions) { index -> listOf(LinearTerm(vector[index], scalar(1.0))) },
    )
}

/**
 * Linear vector expression in Gurobi decision variables.
 *
 * This is used for expressions such as `self.u - other.u`.  Each component stores the Gurobi
 * variables and runtime coefficients that contribute to that dimension, so [dot] can translate the
 * vector directly into an affine expression.
 */
class DecisionVectorExpression internal constructor(
    internal val dimensions: Int,
    internal val components: List<List<LinearTerm>>,
) {
    init {
        require(components.size == dimensions) {
            "Decision vector expression dimension mismatch: ${components.size} != $dimensions"
        }
    }
}

/**
 * Subtracts another decision vector from this one as a symbolic vector expression.
 */
operator fun DecisionVector.minus(other: DecisionVector): DecisionVectorExpression =
    asExpression().combine(other.asExpression()) { left, right ->
        left + right.map { it.copy(coefficient = -it.coefficient) }
    }

private fun DecisionVectorExpression.combine(
    other: DecisionVectorExpression,
    component: (List<LinearTerm>, List<LinearTerm>) -> List<LinearTerm>,
): DecisionVectorExpression {
    require(dimensions == other.dimensions) {
        "Decision vector dimension mismatch: $dimensions != ${other.dimensions}"
    }
    return DecisionVectorExpression(
        dimensions,
        components.zip(other.components).map { (left, right) ->
            component(left, right)
        },
    )
}
