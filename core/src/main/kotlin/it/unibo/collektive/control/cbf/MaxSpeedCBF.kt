package it.unibo.collektive.control.cbf

import it.unibo.collektive.control.dsl.ConstraintFormula
import it.unibo.collektive.control.dsl.ControlFunctionScope

/**
 * Maximum-speed constraint enforced as a quadratic barrier.
 *
 * Formula installed once:
 * ```
 * ‖u_i‖² ≤ u_max²
 * ```
 *
 * The quadratic structure `‖u‖²` is immutable after install.  Only the RHS `u_max²` may change if
 * the robot's maximum speed is adjusted between iterations; this is updated via the [GRBQConstr]
 * RHS attribute setter — no structural changes are needed.
 *
 * @property eta unused (kept for interface compatibility with [CBF])
 * @property slackWeight always `null`; slack on a quadratic norm constraint requires a quadratic
 *  addition to the LHS which is not supported after [GRBModel.addQConstr].
 *  Use variable bounds on the decision vector as an alternative soft limit if needed.
 */
class MaxSpeedCBF(override val eta: Double = 1.0, override val slackWeight: Double? = null) : CBF() {

    override val name: String = "max_speed"

    override val constraintName: String = "u_norm_sq"

    override fun ControlFunctionScope.formula(): ConstraintFormula =
        squaredNorm(self.u) lessThanOrEqualTo squared(self.maxSpeed)
}
