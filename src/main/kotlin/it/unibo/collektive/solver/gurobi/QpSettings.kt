package it.unibo.collektive.solver.gurobi

import it.unibo.collektive.admm.Tolerance

interface QpSettings {
    val constraintPrefix: String
    val logEnabled: Boolean
    val rhoADMM: Double
    val rhoResidual: Double
    val rhoSlack: Double
    val tolerance: Tolerance
}
