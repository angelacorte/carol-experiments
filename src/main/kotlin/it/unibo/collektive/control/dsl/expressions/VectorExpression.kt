package it.unibo.collektive.control.dsl.expressions

import it.unibo.collektive.control.dsl.FormulaRuntime

/**
 * Runtime vector expression used for state-dependent coefficients.
 *
 * Unlike [DecisionVector], this does not represent Gurobi variables.  It is evaluated during
 * constraint updates and is typically used for quantities such as relative positions or target
 * offsets that become coefficients in affine expressions.
 */
class VectorExpression internal constructor(internal val evaluate: (FormulaRuntime) -> DoubleArray)

/**
 * Squared Euclidean norm of a runtime vector.
 */
fun squaredNorm(vector: VectorExpression): RuntimeScalar =
    RuntimeScalar { runtime -> vector.evaluate(runtime).sumOf { it * it } }

/**
 * Adds two runtime vector expressions component by component.
 */
operator fun VectorExpression.plus(other: VectorExpression): VectorExpression = VectorExpression { runtime ->
    val left = evaluate(runtime)
    val right = other.evaluate(runtime)
    require(left.size == right.size) { "Vector dimension mismatch: ${left.size} != ${right.size}" }
    DoubleArray(left.size) { i -> left[i] + right[i] }
}

/**
 * Subtracts another runtime vector expression from this one component by component.
 */
operator fun VectorExpression.minus(other: VectorExpression): VectorExpression = VectorExpression { runtime ->
    val left = evaluate(runtime)
    val right = other.evaluate(runtime)
    require(left.size == right.size) { "Vector dimension mismatch: ${left.size} != ${right.size}" }
    DoubleArray(left.size) { i -> left[i] - right[i] }
}

/**
 * Negates this runtime vector expression component by component.
 */
operator fun VectorExpression.unaryMinus(): VectorExpression =
    VectorExpression { runtime -> evaluate(runtime).map { -it }.toDoubleArray() }

/**
 * Multiplies this runtime vector expression by a runtime scalar.
 */
operator fun RuntimeScalar.times(vector: VectorExpression): VectorExpression = VectorExpression { runtime ->
    val factor = evaluate(runtime)
    vector.evaluate(runtime).map { factor * it }.toDoubleArray()
}

/**
 * Multiplies a runtime vector expression by this constant number.
 */
operator fun Number.times(vector: VectorExpression): VectorExpression = asRuntimeScalar() * vector
