package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRB

enum class FormulaSense(internal val gurobiSense: Char) {
    LessThanOrEqualTo(GRB.LESS_EQUAL),
    GreaterThanOrEqualTo(GRB.GREATER_EQUAL),
}

sealed interface ConstraintFormula {
    val sense: FormulaSense
    val rightHandSide: RuntimeScalar
}

class LinearConstraintFormula internal constructor(
    val leftHandSide: AffineExpression,
    override val sense: FormulaSense,
    override val rightHandSide: RuntimeScalar,
) : ConstraintFormula

class QuadraticConstraintFormula internal constructor(
    val leftHandSide: QuadraticExpression,
    override val sense: FormulaSense,
    override val rightHandSide: RuntimeScalar,
) : ConstraintFormula

infix fun AffineExpression.lessThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
    LinearConstraintFormula(this, FormulaSense.LessThanOrEqualTo, rightHandSide)

infix fun AffineExpression.lessThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this lessThanOrEqualTo scalar(rightHandSide)

infix fun AffineExpression.greaterThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
    LinearConstraintFormula(this, FormulaSense.GreaterThanOrEqualTo, rightHandSide)

infix fun AffineExpression.greaterThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this greaterThanOrEqualTo scalar(rightHandSide)

infix fun QuadraticExpression.lessThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
    QuadraticConstraintFormula(this, FormulaSense.LessThanOrEqualTo, rightHandSide)

infix fun QuadraticExpression.lessThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this lessThanOrEqualTo scalar(rightHandSide)

infix fun QuadraticExpression.greaterThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
    QuadraticConstraintFormula(this, FormulaSense.GreaterThanOrEqualTo, rightHandSide)

infix fun QuadraticExpression.greaterThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this greaterThanOrEqualTo scalar(rightHandSide)
