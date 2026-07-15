package it.unibo.collektive.control.cbf

import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.dsl.ConstraintFormula
import it.unibo.collektive.control.dsl.ControlFunctionScope
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Obstacle

/**
 * Obstacle-avoidance barrier under ZOH dynamics.
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
    private var obstacleProvider: () -> Obstacle,
    override val eta: Double = 0.5,
    override val slackWeight: Double? = null,
    identifier: String? = null,
) : CBF() {

    private val obstacleIdentifier: String =
        identifier ?: obstacleProvider().let { obstacle -> "(${obstacle.x}, ${obstacle.y})" }

    override val name: String = "obstacle_avoidance_CBF_$obstacleIdentifier"

    override fun syncFrom(other: ControlFunction) {
        if (other is ObstacleAvoidanceCBF) {
            obstacleProvider = other.obstacleProvider
        }
    }

    override fun ControlFunctionScope.formula(): ConstraintFormula {
        val obstaclePosition = vector { obstacleProvider().toDoubleArray() }
        val obstacleRadius = scalar {
            obstacleProvider().let { obstacle -> obstacle.radius + obstacle.margin }
        }
        val distance = self.position - obstaclePosition
        val h = squaredNorm(distance) - squared(obstacleRadius)
        return 2.0 * dot(distance, self.u) + slack greaterThanOrEqualTo
            -(eta / timeStep) * h
    }
}
