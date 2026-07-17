package it.unibo.collektive.control.dsl

import it.unibo.collektive.control.dsl.expressions.AffineExpression
import it.unibo.collektive.control.dsl.expressions.QuadraticExpression
import it.unibo.collektive.control.dsl.expressions.RuntimeScalar

/**
 * Common contract for a formula that can be installed as one Gurobi constraint.
 *
 * The left-hand side is represented by concrete subtypes because linear and quadratic constraints
 * are installed through different Gurobi APIs.  The right-hand side is always a runtime scalar so it
 * can be recomputed without rebuilding the model.
 *
 * Formulas are built with the infix constraint builders exposed by [ControlFunctionScope]
 * (`lessThanOrEqualTo` / `greaterThanOrEqualTo`).
 */
sealed interface ConstraintFormula {
    /** Gurobi direction of the inequality represented by this formula. */
    val gurobiSense: Char

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

    /** Gurobi direction of the linear inequality. */
    override val gurobiSense: Char,

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

    /** Gurobi direction of the quadratic inequality. */
    override val gurobiSense: Char,

    /** Runtime scalar installed as the right-hand side of the quadratic inequality. */
    override val rightHandSide: RuntimeScalar,
) : ConstraintFormula
