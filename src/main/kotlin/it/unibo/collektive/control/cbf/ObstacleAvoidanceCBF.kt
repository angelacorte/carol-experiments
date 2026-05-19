package it.unibo.collektive.control.cbf

import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.dsl.FormulaScope
import it.unibo.collektive.control.dsl.div
import it.unibo.collektive.control.dsl.dot
import it.unibo.collektive.control.dsl.greaterThanOrEqualTo
import it.unibo.collektive.control.dsl.minus
import it.unibo.collektive.control.dsl.plus
import it.unibo.collektive.control.dsl.squared
import it.unibo.collektive.control.dsl.squaredNorm
import it.unibo.collektive.control.dsl.times
import it.unibo.collektive.control.dsl.unaryMinus
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Obstacle

/**
 * Static obstacle-avoidance barrier under ZOH dynamics.
 *
 * Discrete-time CBF constraint (installed once, updated every iteration):
 * ```
 * 2(p_i − p_o)ᵀ u_i + slack ≥ −(η/Δt) · h_obs
 * ```
 * where `h_obs = ‖p_i − p_o‖² − (r_o + d_o)²`.
 *
 * The obstacle and robot positions may change across iterations, so the current obstacle is
 * retrieved through [obstacleProvider] during every update and the numerical values are refreshed
 * via [GRBModel.chgCoeff].
 *
 * @property obstacleProvider supplies the current obstacle to avoid
 * @property eta        decay-rate parameter
 * @property slackWeight penalty for the soft version; `null` → hard constraint
 */
class ObstacleAvoidanceCBF(
    override val eta: Double = 0.5,
    override val slackWeight: Double? = null,
    private var obstacleProvider: () -> Obstacle,
) : CBF() {

    override val name: String = "obstacle_avoidance_CBF"

    override fun syncFrom(other: ControlFunction) {
        if (other is ObstacleAvoidanceCBF) {
            obstacleProvider = other.obstacleProvider
        }
    }

    protected override fun FormulaScope.formula() = local {
        val obstaclePosition = vector { obstacleProvider().toDoubleArray() }
        val obstacleRadius = scalar {
            obstacleProvider().radius + obstacleProvider().margin
        }
        val distance = self.position - obstaclePosition
        val h = squaredNorm(distance) - squared(obstacleRadius)

        2.0 * dot(distance, self.u) + slack greaterThanOrEqualTo
            -(eta / timeStep) * h
    }
}
