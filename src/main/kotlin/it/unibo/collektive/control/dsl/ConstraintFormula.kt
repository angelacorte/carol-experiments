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
    /** Formula relation `left <= right`. */
    LessThanOrEqualTo(GRB.LESS_EQUAL),

    /** Formula relation `left >= right`. */
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
    /** Direction of the inequality represented by this formula. */
    val sense: FormulaSense

    /** Scalar expression evaluated as the right-hand side of the installed constraint. */
    val rightHandSide: RuntimeScalar
}

/**
 * Formula whose left-hand side is affine in the Gurobi decision variables.
 *
 * Dynamic coefficients and constants are allowed: they are evaluated from [FormulaRuntime] and
 * pushed into the installed `GRBConstr` via `chgCoeff` and `RHS` updates.
 */
class LinearConstraintFormula internal constructor(
    /** Affine expression installed as the left-hand side of a linear Gurobi constraint. */
    val leftHandSide: AffineExpression,

    /** Direction of the linear inequality. */
    override val sense: FormulaSense,

    /** Runtime scalar installed as the right-hand side of the linear inequality. */
    override val rightHandSide: RuntimeScalar,
) : ConstraintFormula

/**
 * Formula whose left-hand side is quadratic in the Gurobi decision variables.
 *
 * The quadratic structure is fixed at installation time.  The current implementation supports a
 * runtime right-hand side, which is enough for constraints such as maximum speed norms.
 */
class QuadraticConstraintFormula internal constructor(
    /** Quadratic expression installed as the left-hand side of a quadratic Gurobi constraint. */
    val leftHandSide: QuadraticExpression,

    /** Direction of the quadratic inequality. */
    override val sense: FormulaSense,

    /** Runtime scalar installed as the right-hand side of the quadratic inequality. */
    override val rightHandSide: RuntimeScalar,
) : ConstraintFormula

/**
 * Builds a linear formula `this <= rightHandSide`.
 */
infix fun AffineExpression.lessThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
    LinearConstraintFormula(this, FormulaSense.LessThanOrEqualTo, rightHandSide)

/**
 * Builds a linear formula `this <= rightHandSide` with a constant right-hand side.
 */
infix fun AffineExpression.lessThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this lessThanOrEqualTo scalar(rightHandSide)

/**
 * Builds a linear formula `this >= rightHandSide`.
 */
infix fun AffineExpression.greaterThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
    LinearConstraintFormula(this, FormulaSense.GreaterThanOrEqualTo, rightHandSide)

/**
 * Builds a linear formula `this >= rightHandSide` with a constant right-hand side.
 */
infix fun AffineExpression.greaterThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this greaterThanOrEqualTo scalar(rightHandSide)

/**
 * Builds a quadratic formula `this <= rightHandSide`.
 */
infix fun QuadraticExpression.lessThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
    QuadraticConstraintFormula(this, FormulaSense.LessThanOrEqualTo, rightHandSide)

/**
 * Builds a quadratic formula `this <= rightHandSide` with a constant right-hand side.
 */
infix fun QuadraticExpression.lessThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this lessThanOrEqualTo scalar(rightHandSide)

/**
 * Builds a quadratic formula `this >= rightHandSide`.
 */
infix fun QuadraticExpression.greaterThanOrEqualTo(rightHandSide: RuntimeScalar): ConstraintFormula =
    QuadraticConstraintFormula(this, FormulaSense.GreaterThanOrEqualTo, rightHandSide)

/**
 * Builds a quadratic formula `this >= rightHandSide` with a constant right-hand side.
 */
infix fun QuadraticExpression.greaterThanOrEqualTo(rightHandSide: Double): ConstraintFormula =
    this greaterThanOrEqualTo scalar(rightHandSide)
