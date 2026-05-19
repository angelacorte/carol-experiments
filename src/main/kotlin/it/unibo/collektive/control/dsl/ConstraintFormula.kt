package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRB
import it.unibo.collektive.control.dsl.expressions.AffineExpression
import it.unibo.collektive.control.dsl.expressions.QuadraticExpression
import it.unibo.collektive.control.dsl.expressions.RuntimeScalar
import it.unibo.collektive.control.dsl.expressions.scalar

/**
 * Inequality direction supported by formula-backed constraints.
 *
 * The enum deliberately mirrors only the senses currently used by CBF/CLF constraints.  Each value
 * stores the corresponding Gurobi sense used when the formula is compiled.
 */
enum class FormulaSense(internal val gurobiSense: Char) {
    LessThanOrEqualTo(GRB.LESS_EQUAL),
    GreaterThanOrEqualTo(GRB.GREATER_EQUAL),
}

/**
 * Common contract for a formula that can be installed as one Gurobi constraint.
 *
 * The left-hand side is represented by concrete subtypes because linear and quadratic constraints
 * are installed through different Gurobi APIs.  The right-hand side is always a runtime scalar so it
 * can be recomputed without rebuilding the model.
 */
sealed interface ConstraintFormula {
    val sense: FormulaSense
    val rightHandSide: RuntimeScalar
}

/**
 * Formula whose left-hand side is affine in the Gurobi decision variables.
 *
 * Dynamic coefficients and constants are allowed: they are evaluated from [FormulaRuntime] and
 * pushed into the installed `GRBConstr` via `chgCoeff` and `RHS` updates.
 */
class LinearConstraintFormula internal constructor(
    val leftHandSide: AffineExpression,
    override val sense: FormulaSense,
    override val rightHandSide: RuntimeScalar,
) : ConstraintFormula

/**
 * Formula whose left-hand side is quadratic in the Gurobi decision variables.
 *
 * The quadratic structure is fixed at installation time.  The current implementation supports a
 * runtime right-hand side, which is enough for constraints such as maximum speed norms.
 */
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
