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
    val linearTerms = decision.terms.flatMap { decisionTerm ->
        val dimension = decisionTerm.vector.dimensions
        List(dimension) { index ->
            LinearTerm(
                variable = decisionTerm.vector.vector[index],
                coefficient = RuntimeScalar { runtime ->
                    val values = coefficients.evaluate(runtime)
                    require(values.size == dimension) {
                        "Coefficient dimension mismatch: ${values.size} != $dimension"
                    }
                    values[index] * decisionTerm.coefficient.evaluate(runtime)
                },
            )
        }
    }
    return AffineExpression(linearTerms)
}
