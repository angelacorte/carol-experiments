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

    internal fun asExpression(): DecisionVectorExpression =
        DecisionVectorExpression(listOf(DecisionVectorTerm(scalar(1.0), this)))
}

/**
 * One scaled decision vector inside a [DecisionVectorExpression].
 */
internal data class DecisionVectorTerm(val coefficient: RuntimeScalar, val vector: DecisionVector)

/**
 * Linear combination of decision vectors.
 *
 * This is used for expressions such as `self.u - other.u`.  It is not a Gurobi expression yet; it is
 * an intermediate representation that becomes concrete linear terms once paired with coefficients
 * through [dot].
 */
class DecisionVectorExpression internal constructor(internal val terms: List<DecisionVectorTerm>)

/**
 * Adds two decision vectors as a symbolic vector expression.
 */
operator fun DecisionVector.plus(other: DecisionVector): DecisionVectorExpression =
    asExpression() + other.asExpression()

/**
 * Subtracts another decision vector from this one as a symbolic vector expression.
 */
operator fun DecisionVector.minus(other: DecisionVector): DecisionVectorExpression =
    asExpression() - other.asExpression()

/**
 * Negates this decision vector as a symbolic vector expression.
 */
operator fun DecisionVector.unaryMinus(): DecisionVectorExpression = -asExpression()

/**
 * Adds two symbolic linear combinations of decision vectors.
 */
operator fun DecisionVectorExpression.plus(other: DecisionVectorExpression): DecisionVectorExpression =
    DecisionVectorExpression(terms + other.terms)

/**
 * Adds a decision vector to a symbolic linear combination of decision vectors.
 */
operator fun DecisionVectorExpression.plus(other: DecisionVector): DecisionVectorExpression =
    this + other.asExpression()

/**
 * Subtracts one symbolic linear combination of decision vectors from another.
 */
operator fun DecisionVectorExpression.minus(other: DecisionVectorExpression): DecisionVectorExpression = this + (-other)

/**
 * Subtracts a decision vector from a symbolic linear combination of decision vectors.
 */
operator fun DecisionVectorExpression.minus(other: DecisionVector): DecisionVectorExpression =
    this - other.asExpression()

/**
 * Negates every coefficient in this symbolic linear combination of decision vectors.
 */
operator fun DecisionVectorExpression.unaryMinus(): DecisionVectorExpression =
    DecisionVectorExpression(terms.map { DecisionVectorTerm(-it.coefficient, it.vector) })

/**
 * Scales a symbolic linear combination of decision vectors by a runtime scalar.
 */
operator fun RuntimeScalar.times(decision: DecisionVectorExpression): DecisionVectorExpression =
    DecisionVectorExpression(decision.terms.map { it.copy(coefficient = it.coefficient * this) })

/**
 * Scales a symbolic linear combination of decision vectors by a constant number.
 */
operator fun Number.times(decision: DecisionVectorExpression): DecisionVectorExpression = asRuntimeScalar() * decision

/**
 * Scales a decision vector by a runtime scalar.
 */
operator fun RuntimeScalar.times(decision: DecisionVector): DecisionVectorExpression = this * decision.asExpression()

/**
 * Scales a decision vector by a constant number.
 */
operator fun Number.times(decision: DecisionVector): DecisionVectorExpression = asRuntimeScalar() * decision
