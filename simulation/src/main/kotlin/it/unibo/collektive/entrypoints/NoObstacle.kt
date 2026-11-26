package it.unibo.collektive.entrypoints

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.admm.admmEntrypoint
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.alchemist.device.controlPeriodMillis
import it.unibo.collektive.alchemist.device.getRobot
import it.unibo.collektive.alchemist.device.getTarget
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.alchemist.device.sensors.TimeSensor
import it.unibo.collektive.alchemist.device.targetId
import it.unibo.collektive.control.GoToTargetNominal
import it.unibo.collektive.control.cbf.CollisionAvoidanceCBF
import it.unibo.collektive.control.cbf.MaxSpeedCBF
import it.unibo.collektive.control.clf.GoToTargetCLF
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.solver.Solver

/**
 * Main aggregate entrypoint: runs distributed ADMM to compute a safe control and applies it when converged.
 */
fun Aggregate<Int>.noObstacleEntrypoint(
    position: LocationSensor,
    timeSensor: TimeSensor,
    device: CollektiveDevice<Euclidean2DPosition>,
    solver: Solver,
) = context(position, device, timeSensor, solver) {
    val robot = getRobot()
    admmEntrypoint(
        device.controlPeriodMillis,
        robot,
        uNominal = GoToTargetNominal { getTarget(device.targetId) }.compute(robot).toDoubleArray(),
        localCLF = listOf(GoToTargetCLF { getTarget(device.targetId) }),
        localCBF = listOf(MaxSpeedCBF()),
        pairwiseCBF = listOf(CollisionAvoidanceCBF()),
    )
}
