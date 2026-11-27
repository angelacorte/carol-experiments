package it.unibo.collektive.alchemist

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.admm.admmEntrypoint
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.alchemist.device.applyControl
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.device.Actuator
import it.unibo.collektive.device.TimeSensor
import it.unibo.collektive.model.Device
import it.unibo.collektive.solver.Solver

/**
 * Runs the core [admmEntrypoint] inside Alchemist: controls move the node in the environment,
 * and the time left before the next control is exported as the `TimeLeft` molecule.
 */
context(
    collektiveDevice: CollektiveDevice<Euclidean2DPosition>,
    locationSensor: LocationSensor,
    solver: Solver,
    timeSensor: TimeSensor,
)
fun Aggregate<Int>.simulationAdmmEntrypoint(
    controlPeriod: Double,
    device: Device,
    uNominal: DoubleArray,
    localCLF: List<CLF>,
    localCBF: List<CBF>,
    pairwiseCBF: List<CBF>,
) = context(Actuator { control, deltaTime -> applyControl(control, deltaTime) }) {
    collektiveDevice["TimeLeft"] = admmEntrypoint(controlPeriod, device, uNominal, localCLF, localCBF, pairwiseCBF)
}
