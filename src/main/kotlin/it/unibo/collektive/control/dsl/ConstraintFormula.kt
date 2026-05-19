package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRB

enum class FormulaSense(internal val gurobiSense: Char) {
    LessThanOrEqualTo(GRB.LESS_EQUAL),
    GreaterThanOrEqualTo(GRB.GREATER_EQUAL),
}

sealed interface ConstraintFormula {
    val sense: FormulaSense
    val rightHandSide: ScalarExpression
}

class LinearConstraintFormula internal constructor(
    val leftHandSide: AffineExpression,
    override val sense: FormulaSense,
    override val rightHandSide: ScalarExpression,
) : ConstraintFormula

class QuadraticConstraintFormula internal constructor(
    val leftHandSide: QuadraticExpression,
    override val sense: FormulaSense,
    override val rightHandSide: ScalarExpression,
) : ConstraintFormula

infix fun AffineExpression.lessThanOrEqualTo(rightHandSide: ScalarExpression): ConstraintFormula =
    LinearConstraintFormula(this, FormulaSense.LessThanOrEqualTo, rightHandSide)

infix fun AffineExpression.lessThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this lessThanOrEqualTo scalar(rightHandSide)

infix fun AffineExpression.greaterThanOrEqualTo(rightHandSide: ScalarExpression): ConstraintFormula =
    LinearConstraintFormula(this, FormulaSense.GreaterThanOrEqualTo, rightHandSide)

infix fun AffineExpression.greaterThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this greaterThanOrEqualTo scalar(rightHandSide)

infix fun QuadraticExpression.lessThanOrEqualTo(rightHandSide: ScalarExpression): ConstraintFormula =
    QuadraticConstraintFormula(this, FormulaSense.LessThanOrEqualTo, rightHandSide)

infix fun QuadraticExpression.lessThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this lessThanOrEqualTo scalar(rightHandSide)

infix fun QuadraticExpression.greaterThanOrEqualTo(rightHandSide: ScalarExpression): ConstraintFormula =
    QuadraticConstraintFormula(this, FormulaSense.GreaterThanOrEqualTo, rightHandSide)

infix fun QuadraticExpression.greaterThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this greaterThanOrEqualTo scalar(rightHandSide)
