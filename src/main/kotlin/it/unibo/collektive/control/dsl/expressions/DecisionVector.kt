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

operator fun DecisionVector.plus(other: DecisionVector): DecisionVectorExpression =
    asExpression() + other.asExpression()

operator fun DecisionVector.minus(other: DecisionVector): DecisionVectorExpression =
    asExpression() - other.asExpression()

operator fun DecisionVector.unaryMinus(): DecisionVectorExpression = -asExpression()

operator fun DecisionVectorExpression.plus(other: DecisionVectorExpression): DecisionVectorExpression =
    DecisionVectorExpression(terms + other.terms)

operator fun DecisionVectorExpression.plus(other: DecisionVector): DecisionVectorExpression =
    this + other.asExpression()

operator fun DecisionVectorExpression.minus(other: DecisionVectorExpression): DecisionVectorExpression = this + (-other)

operator fun DecisionVectorExpression.minus(other: DecisionVector): DecisionVectorExpression =
    this - other.asExpression()

operator fun DecisionVectorExpression.unaryMinus(): DecisionVectorExpression =
    DecisionVectorExpression(terms.map { DecisionVectorTerm(-it.coefficient, it.vector) })

operator fun RuntimeScalar.times(decision: DecisionVectorExpression): DecisionVectorExpression =
    DecisionVectorExpression(decision.terms.map { it.copy(coefficient = it.coefficient * this) })

operator fun Number.times(decision: DecisionVectorExpression): DecisionVectorExpression = asRuntimeScalar() * decision

operator fun RuntimeScalar.times(decision: DecisionVector): DecisionVectorExpression = this * decision.asExpression()

operator fun Number.times(decision: DecisionVector): DecisionVectorExpression = asRuntimeScalar() * decision
