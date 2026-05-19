package it.unibo.collektive.control.dsl

import it.unibo.collektive.model.Device
import it.unibo.collektive.solver.gurobi.QpSettings

/**
 * Values available while refreshing an already-installed formula constraint.
 */
data class FormulaRuntime(
    val self: Device,
    val other: Device?,
    val settings: QpSettings,
    val deltaTime: Double,
)

