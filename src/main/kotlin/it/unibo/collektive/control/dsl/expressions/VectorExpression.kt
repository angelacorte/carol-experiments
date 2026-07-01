package it.unibo.collektive.control.dsl.expressions

import it.unibo.collektive.control.dsl.FormulaRuntime

/**
 * Runtime vector expression used for state-dependent coefficients.
 *
 * Unlike [DecisionVector], this does not represent Gurobi variables.  It is evaluated during
 * constraint updates and is typically used for quantities such as relative positions or target
 * offsets that become coefficients in affine expressions.
 */
class VectorExpression internal constructor(private val expression: RuntimeExpression<DoubleArray>) {
    internal fun evaluate(runtime: FormulaRuntime): DoubleArray = expression.evaluate(runtime)
}

/**
 * Squared Euclidean norm of a runtime vector.
 */
fun squaredNorm(vector: VectorExpression): RuntimeScalar =
    RuntimeScalar { runtime -> vector.evaluate(runtime).sumOf { it * it } }

/**
 * Subtracts another runtime vector expression from this one component by component.
 */
operator fun VectorExpression.minus(other: VectorExpression): VectorExpression = VectorExpression { runtime ->
    val left = evaluate(runtime)
    val right = other.evaluate(runtime)
    require(left.size == right.size) { "Vector dimension mismatch: ${left.size} != ${right.size}" }
    DoubleArray(left.size) { i -> left[i] - right[i] }
}
