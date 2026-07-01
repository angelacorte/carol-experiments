package it.unibo.collektive.control.dsl.expressions

/**
 * Builds an affine dot product between runtime coefficients and one decision vector.
 */
fun dot(coefficients: VectorExpression, decision: DecisionVector): AffineExpression =
    dot(coefficients, decision.asExpression())

/**
 * Builds an affine dot product between runtime coefficients and a linear combination of decisions.
 *
 * The resulting expression is affine in Gurobi variables, while the coefficients can still depend on
 * the current runtime state.
 */
fun dot(coefficients: VectorExpression, decision: DecisionVectorExpression): AffineExpression {
    val linearTerms = decision.components.flatMapIndexed { index, componentTerms ->
        componentTerms.map { term ->
            term.copy(
                coefficient = RuntimeScalar { runtime ->
                    val values = coefficients.evaluate(runtime)
                    require(values.size == decision.dimensions) {
                        "Coefficient dimension mismatch: ${values.size} != ${decision.dimensions}"
                    }
                    values[index] * term.coefficient.evaluate(runtime)
                },
            )
        }
    }
    return AffineExpression(linearTerms)
}
