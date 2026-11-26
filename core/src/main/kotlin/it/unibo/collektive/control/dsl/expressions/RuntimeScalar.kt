package it.unibo.collektive.control.dsl.expressions

import it.unibo.collektive.control.dsl.FormulaRuntime

/**
 * Runtime expression evaluated against the current formula update context.
 */
internal fun interface RuntimeExpression<out T> {
    fun evaluate(runtime: FormulaRuntime): T
}

/**
 * Scalar value that is evaluated against the current [FormulaRuntime].
 *
 * This type is the scalar building block of the DSL.  It represents both constant numbers and
 * values that change at every solver iteration, such as `timeStep`, `self.maxSpeed`, or an obstacle
 * radius.  Formula compilation stores these expressions and evaluates them during
 * [it.unibo.collektive.solver.gurobi.InstalledControlConstraint.update].
 *
 * All arithmetic operators are member functions, so formulas can combine scalars without imports.
 */
class RuntimeScalar internal constructor(private val expression: RuntimeExpression<Double>) {
    internal fun evaluate(runtime: FormulaRuntime): Double = expression.evaluate(runtime)

    /** Adds two runtime scalars. */
    operator fun plus(other: RuntimeScalar): RuntimeScalar = RuntimeScalar { evaluate(it) + other.evaluate(it) }

    /** Subtracts another runtime scalar from this one. */
    operator fun minus(other: RuntimeScalar): RuntimeScalar = RuntimeScalar { evaluate(it) - other.evaluate(it) }

    /** Negates this runtime scalar. */
    operator fun unaryMinus(): RuntimeScalar = RuntimeScalar { -evaluate(it) }

    /** Multiplies two runtime scalars. */
    operator fun times(other: RuntimeScalar): RuntimeScalar = RuntimeScalar { evaluate(it) * other.evaluate(it) }

    /** Scales every coefficient and the constant part of [affine] by this runtime scalar. */
    operator fun times(affine: AffineExpression): AffineExpression = AffineExpression(
        affine.terms.map { it.copy(coefficient = it.coefficient * this) },
        affine.constant * this,
    )

    /** Divides this runtime scalar by another runtime scalar. */
    operator fun div(other: RuntimeScalar): RuntimeScalar = RuntimeScalar { evaluate(it) / other.evaluate(it) }
}

/**
 * Creates a constant runtime scalar.
 *
 * Constants are represented with the same type as dynamic values so formulas can freely combine
 * literal coefficients and state-dependent quantities.
 */
internal fun scalar(value: Double): RuntimeScalar = RuntimeScalar { value }

/**
 * Converts any numeric literal into the scalar expression domain.
 */
internal fun Number.asRuntimeScalar(): RuntimeScalar = scalar(toDouble())
