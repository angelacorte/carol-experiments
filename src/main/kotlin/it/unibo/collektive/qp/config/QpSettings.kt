package it.unibo.collektive.qp.config

/**
 * Centralized tuning parameters for QP/CBF formulations.
 */
data class QpSettings(
    val rhoSlack: Double = 2.0,
    val rhoAdmm: Double = 10.0,
    val gammaCollision: Double = 0.5,
    val gammaComm: Double = 0.5,
    val gammaObstacle: Double = 0.5,
    val convergenceRate: Double = 1.0,
    val logEnabled: Boolean = false,
    val constraintPrefix: String = "qp",
)

