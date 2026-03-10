package it.unibo.collektive.qp.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.carol.DualParams
import it.unibo.collektive.qp.carol.IncidentDuals
import it.unibo.collektive.qp.carol.SuggestedControl
import it.unibo.collektive.qp.config.QpSettings
import it.unibo.collektive.qp.controlFunctions.CbfContext
import it.unibo.collektive.qp.controlFunctions.applyLocalCbfs
import it.unibo.collektive.qp.controlFunctions.applyPairwiseCbfs
import it.unibo.collektive.qp.controlFunctions.goToTargetCLF
import it.unibo.collektive.qp.controlFunctions.maxSpeedCBF
import it.unibo.collektive.qp.controlFunctions.minimizeADMMCommonQP
import it.unibo.collektive.qp.controlFunctions.minimizeADMMLocalQP
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.toDoubleArray

// Shared setup for local ADMM QPs; guarantees model lifecycle is handled consistently.
private fun <T> withLocalAdmmModel(
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    settings: QpSettings = QpSettings(),
    block: (model: GRBModel, u: GRBVector, delta: GRBVar, position: DoubleArray) -> T,
): T = withModel(settings, "local") { model ->
    val u: GRBVector = model.addVecVar(
        dimension = robot.position.dimension,
        lowerBound = -robot.maxSpeed,
        upperBound = robot.maxSpeed,
        name = "u",
    )
    val delta = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, ConstraintNames.slack("local"))
    val position: DoubleArray = robot.toDoubleArray()
    block(model, u, delta, position)
}

/**
 * Solves the local QP that moves the robot toward the target while
 * avoiding a single obstacle and enforcing ADMM consensus.
 *
 * @return optimal control and slack value for the local agent.
 */
fun avoidObstacleGoToTarget(
    robot: Robot,
    target: Target,
    obstacle: Obstacle? = null,
    average: DoubleArray,
    cardinality: Int,
    settings: QpSettings = QpSettings(),
): Pair<SpeedControl2D, Double> = withLocalAdmmModel(robot, target, obstacle, settings) { model, u, delta, _ ->
    applyLocalCbfs(model, u, CbfContext(self = robot, obstacle = obstacle, settings = settings))
    model.maxSpeedCBF(u, robot)
    model.goToTargetCLF(target, robot.toDoubleArray(), u, delta, settings)
    model.minimizeADMMLocalQP(u, delta, robot, target, average, cardinality, settings)
}

/**
 * Solves the local QP that moves the robot toward the target while
 * avoiding a single obstacle and enforcing ADMM consensus.
 *
 * @return optimal control and slack value for the local agent.
 */
fun <ID : Comparable<ID>> avoidObstacleGoToTarget(
    robot: Robot,
    target: Target,
    obstacle: Obstacle?,
    duals: Map<ID, DualParams>,
    settings: QpSettings = QpSettings(),
): Pair<SpeedControl2D, Double> = withLocalAdmmModel(robot, target, obstacle, settings) { model, u, delta, _ ->
    applyLocalCbfs(model, u, CbfContext(self = robot, obstacle = obstacle, settings = settings))
    model.maxSpeedCBF(u, robot)
    model.goToTargetCLF(target, robot.toDoubleArray(), u, delta, settings)
    model.minimizeADMMLocalQP(u, delta, robot, target, duals, settings)
}

/**
 * Solves the pairwise QP that enforces robot avoidance (and optionally communication range) for an edge.
 */
fun robotAvoidanceAndCommunicationRangeCBF(
    robot: Robot,
    other: Robot,
    range: Double? = null,
    incidentDuals: IncidentDuals,
    settings: QpSettings = QpSettings(),
): SuggestedControl = withModel(settings, "pairwise") { model ->
    val zi: GRBVector = model.addVecVar(
        dimension = robot.position.dimension,
        lowerBound = -robot.maxSpeed,
        upperBound = robot.maxSpeed,
        name = "z_ij^i",
    )
    val zj: GRBVector = model.addVecVar(
        dimension = other.position.dimension,
        lowerBound = -other.maxSpeed,
        upperBound = other.maxSpeed,
        name = "z_ij^j",
    )
    applyPairwiseCbfs(
        model,
        zi,
        zj,
        CbfContext(self = robot, other = other, communicationRange = range, settings = settings),
    )
    model.minimizeADMMCommonQP(zi, zj, robot, other, incidentDuals, settings)
}
