package it.unibo.collektive.control.cbf

import it.unibo.collektive.control.dsl.ConstraintFormula
import it.unibo.collektive.control.dsl.ControlFunctionScope
import it.unibo.collektive.control.dsl.greaterThanOrEqualTo
import it.unibo.collektive.control.dsl.expressions.div
import it.unibo.collektive.control.dsl.expressions.dot
import it.unibo.collektive.control.dsl.expressions.max
import it.unibo.collektive.control.dsl.expressions.minus
import it.unibo.collektive.control.dsl.expressions.plus
import it.unibo.collektive.control.dsl.expressions.squared
import it.unibo.collektive.control.dsl.expressions.squaredNorm
import it.unibo.collektive.control.dsl.expressions.times
import it.unibo.collektive.control.dsl.expressions.unaryMinus

/**
 * Robot–robot collision avoidance barrier under ZOH dynamics.
 *
 * Discrete-time CBF constraint (installed once, updated every iteration):
 * ```
 * 2(p_i - p_j)ᵀ(u_i - u_j) + slack ≥ −(η/Δt) · h_col
 * ```
 * where `h_col = ‖p_i − p_j‖² − d_min²`.
 *
 * The LHS coefficients `2(p_i − p_j)[k]` and the RHS `−(η/Δt)·h_col` change every step as the
 * device move.  [GRBModel.chgCoeff] is used to update them in-place without rebuilding the model.
 *
 * @property eta        decay-rate parameter
 * @property slackWeight objective penalty for the slack variable; `null` → hard constraint (no slack)
 */
class CollisionAvoidanceCBF(override val eta: Double = 0.5, override val slackWeight: Double? = null) : CBF() {

    override val name: String = "collision_avoidance_CBF"

    override fun ControlFunctionScope.formula(): ConstraintFormula {
        val distance = self.position - other.position
        val minDistance = max(self.safeMargin, other.safeMargin)
        val h = squaredNorm(distance) - squared(minDistance)

        return 2.0 * dot(distance, self.u - other.u) + slack greaterThanOrEqualTo
            -(eta / timeStep) * h
    }
}
