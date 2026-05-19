package it.unibo.collektive.control.cbf

import it.unibo.collektive.control.dsl.FormulaScope
import it.unibo.collektive.control.dsl.div
import it.unibo.collektive.control.dsl.dot
import it.unibo.collektive.control.dsl.greaterThanOrEqualTo
import it.unibo.collektive.control.dsl.max
import it.unibo.collektive.control.dsl.minus
import it.unibo.collektive.control.dsl.plus
import it.unibo.collektive.control.dsl.squared
import it.unibo.collektive.control.dsl.squaredNorm
import it.unibo.collektive.control.dsl.times
import it.unibo.collektive.control.dsl.unaryMinus

/**
 * Communication-range barrier: keeps two robots within [range] of each other.
 *
 * Robustified discrete-time CBF under ZOH dynamics (installed once, updated every iteration):
 * ```
 * −2(p_i − p_j)ᵀ(u_i − u_j) + slack ≥ −(η/Δt)·h_com + 4Δt·u_max²
 * ```
 * where `h_com = R² − ‖p_i − p_j‖²`.
 *
 * Both the LHS coefficients and the RHS change with robot positions and Δt.
 * [GRBModel.chgCoeff] updates them without structural changes.
 *
 * @property range      maximum allowed distance between the two robots
 * @property eta        decay-rate parameter
 * @property slackWeight penalty for the soft version; `null` → hard constraint
 */
class CommunicationRangeCBF(
    private val range: Double,
    override val eta: Double = 0.5,
    override val slackWeight: Double? = null,
) : CBF() {

    override val name: String = "communication_range_CBF"

    protected override fun FormulaScope.formula() = pairwise {
        val distance = self.position - other.position
        val h = squared(range) - squaredNorm(distance)
        val maxSpeed = max(self.maxSpeed, other.maxSpeed)

        -2.0 * dot(distance, self.u - other.u) + slack greaterThanOrEqualTo
            -(eta / timeStep) * h + 4.0 * timeStep * squared(maxSpeed)
    }
}
