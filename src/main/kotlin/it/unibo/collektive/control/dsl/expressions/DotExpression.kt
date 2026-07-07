package it.unibo.collektive.control.dsl.expressions

/**
 * Builds an affine dot product between runtime coefficients and a decision expression.
 *
 * [decision] may be a raw decision vector (`self.u`) or any static combination of decision vectors
 * (`self.u - other.u`): [DecisionExpression] represents both uniformly. The resulting expression is affine
 * in Gurobi variables, while [coefficients] can still depend on the current runtime state.
 */
fun dot(coefficients: VectorExpression, decision: DecisionExpression): AffineExpression {
    val linearTerms = decision.components.flatMapIndexed { index, componentTerms ->
        componentTerms.map { term ->
            LinearTerm(
                term.variable,
                RuntimeScalar { runtime ->
                    val values = coefficients.evaluate(runtime)
                    require(values.size == decision.dimensions) {
                        "Coefficient dimension mismatch: ${values.size} != ${decision.dimensions}"
                    }
                    values[index] * term.coefficient
                },
            )
        }
    }
    return AffineExpression(linearTerms)
}
