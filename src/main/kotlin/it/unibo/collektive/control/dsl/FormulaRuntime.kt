package it.unibo.collektive.control.dsl

import it.unibo.collektive.model.Device
import it.unibo.collektive.solver.gurobi.QpSettings

/**
 * Runtime values used to evaluate a formula after its Gurobi structure has already been installed.
 *
 * A control function formula is built once, when the reusable QP model is created.  Numerical data
 * such as robot positions, the neighbor state, and the current time step are instead read at every
 * solver iteration through this context.  Runtime expressions keep a reference to this object so
 * the installed constraint can update only coefficients and RHS values without rebuilding the model.
 *
 * @property self current state of the local device.
 * @property other current state of the neighbor device, when the formula is pairwise.
 * @property settings solver settings shared by the active QP.
 * @property deltaTime control horizon expressed in seconds.
 */
data class FormulaRuntime(
    val self: Device,
    val other: Device?,
    val settings: QpSettings,
    val deltaTime: Double,
)
