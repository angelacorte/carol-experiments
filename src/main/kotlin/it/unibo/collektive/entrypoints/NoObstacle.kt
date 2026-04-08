package it.unibo.collektive.entrypoints

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.admm.admmEntrypoint
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.alchemist.SimulationSolver.solver
import it.unibo.collektive.alchemist.device.SimulationQpSettings
import it.unibo.collektive.alchemist.device.getRobot
import it.unibo.collektive.alchemist.device.getTarget
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.alchemist.device.sensors.TimeSensor
import it.unibo.collektive.control.GoToTargetNominal
import it.unibo.collektive.control.cbf.CollisionAvoidanceCBF
import it.unibo.collektive.control.cbf.MaxSpeedCBF
import it.unibo.collektive.control.clf.GoToTargetCLF
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Target

/**
 * Main aggregate entrypoint: runs distributed ADMM to compute a safe control and applies it when converged.
 */
fun Aggregate<Int>.noObstacleEntrypoint(
    position: LocationSensor,
    timeSensor: TimeSensor,
    device: CollektiveDevice<Euclidean2DPosition>,
) = context(position, device, timeSensor) {
    val robot = getRobot()
    val target: Target = getTarget(device["TargetID"] as Number)
    admmEntrypoint(
        device["ControlPeriodMS"] as? Double ?: 100.0,
        robot,
        uNominal = GoToTargetNominal { getTarget(device["TargetID"] as Number) }.compute(robot).toDoubleArray(),
        solver = device.environment.solver(SimulationQpSettings().base(device)),
        localCLF = listOf(GoToTargetCLF { getTarget(device["TargetID"] as Number) }),
        localCBF = listOf(MaxSpeedCBF()),
        pairwiseCBF = listOf(CollisionAvoidanceCBF()),
    )
}
