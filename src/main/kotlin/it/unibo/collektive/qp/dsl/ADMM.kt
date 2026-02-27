package it.unibo.collektive.qp.dsl

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.neighboring
import it.unibo.collektive.aggregate.api.sharing
import it.unibo.collektive.aggregate.toMap
import it.unibo.collektive.aggregate.values
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.Vector2D
import it.unibo.collektive.qp.utils.avg
import it.unibo.collektive.qp.utils.getObstacle
import it.unibo.collektive.qp.utils.getRobot
import it.unibo.collektive.qp.utils.getTarget
import it.unibo.collektive.qp.utils.initVector2D
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.norm
import it.unibo.collektive.qp.utils.plus
import it.unibo.collektive.qp.utils.toDoubleArray
import it.unibo.collektive.stdlib.collapse.max
import it.unibo.collektive.stdlib.spreading.gossipMax
import org.apache.commons.lang3.compare.ComparableUtils.min

//data class Coupled(val suggestedControl: SuggestedControl, val residuals: Residual)
//data class Parameters(val robot: Robot, val edges: List<Coupled>)
data class Tolerance(val primal: Double, val dual: Double)

data class LocalUpdate(val yi: Vector2D, val yj: Vector2D) // y_{ij}^{i}, y_{ij}^{j}

data class SuggestedControl(val zi:SpeedControl2D, val zj: SpeedControl2D) // z_{ij} = [z_ij^i z_ij^j]

data class DualParams(val suggestedControl: SuggestedControl, val localUpdate: LocalUpdate)

data class ControlAndResiduals<ID: Comparable<ID>>(val control: SpeedControl2D, val duals: Map<ID, DualParams>)

fun Aggregate<Int>.entrypoint(position: LocationSensor, device: CollektiveDevice<Euclidean2DPosition>) = context(position, device) {
    val tolerance = Tolerance(1e-3, 1e-3) // used the same tolerance as in ADMM Example3
    val iter = 0
    val robot = getRobot(localId)
    val target: Target = getTarget(device["TargetID"] as Number)
    val obstacle = getObstacle()

    val duals = evolve(emptyMap<Int, DualParams>()) { previousDuals ->
        val output: ControlAndResiduals<Int> = admm(robot, target, obstacle, previousDuals)


        // primal residual
        val neighborsExit = neighboring(output)
        val primalResidualLocal = neighborsExit.map<Double> { TODO("||ui - ziji||") }.neighbors.values.max()
        val primalResidual = gossipMax(primalResidualLocal)

        // r_ij^t = max ||ui - zij,i||
        val rijt = output.suggested.maxOfOrNull { (output.control - it.controlForLocal).norm() }
        // r^t = max ri^t
        val rt = gossipMax(rijt ?: 0.0)

        // dual residual
        val dualResidualLocal =
            neighborsExit.map<Double> { TODO("||ziji current - ziji previous||") }.neighbors.values.max()
        val dualResidual = gossipMax(dualResidualLocal)

        val previous: SpeedControl2D = TODO("zij^it-1")
        // sit = \rhoa max ||zij^it - zij^it-1||
        val sit = output.suggested.map { (it.controlForOther - previous).norm() }.max()
        val st = gossipMax(sit)


        output.duals
    }
    // stop if residuals < threshold
}

fun <ID: Comparable<ID>> Aggregate<ID>.admm(
    robot: Robot,
    target: Target,
    obstacle: Obstacle,
    duals: Map<ID, DualParams>,
): ControlAndResiduals<ID> = sharing(robot) { fieldU ->
    val avg: SpeedControl2D = fieldU.map { (_, value) ->
        value.control
    }.neighbors.values.list.avg()
    val control: SpeedControl2D = executeLocalADMM(robot, target, obstacle, avg.toDoubleArray(), fieldU.neighbors.values.size)
    val robotUpdated = robot.copy(control = control)
    val commons: Map<ID, DualParams> = fieldU.map { (id, r) ->
        val localUpdate = duals[id]?.localUpdate ?: LocalUpdate(initVector2D(), initVector2D())
        val (zi, zj) = robotAvoidanceAndCommunicationRangeCBF(robotUpdated, r, 10.0, localUpdate)
        // local dual update
        // y_ij^i,t+1 = y_ij^i,t + ( u_i^t+1 - z_ij^i,t+1) // y_ij^j,t+1 = y_ij^j,t + ( u_j^t+1 - z_ij^j,t+1)
        val newLocalUpdate = LocalUpdate(localUpdate.yi + control - zi, localUpdate.yj + r.control - zj)
        DualParams(SuggestedControl(zi, zj), newLocalUpdate)
    }.neighbors.toMap()
    robotUpdated.yielding{ ControlAndResiduals(control, commons) }
}

fun executeLocalADMM(robot: Robot, target: Target, obstacle: Obstacle, avg: DoubleArray, cardinality: Int): SpeedControl2D {
    val (uWanted, deltaNom) = avoidObstacleGoToTarget(robot, target, obstacle, avg, cardinality)
    return uWanted
}

///**
// * Initialize ADMM parameters for the first iteration of the algorithm, for the given [robot].
// */
//fun <ID: Comparable<ID>> Aggregate<ID>.initParameters(robot: Robot): Parameters {
//    val neighborsVel = neighboring(robot.control)
//    val coupledEdges: List<Coupled> = neighborsVel.neighbors.list.map {
//        Coupled(
//            it.id,
//            SuggestedControl(robot.control, it.value),
//            Residual(initVector2D(),initVector2D())
//        )
//    }
////    val controls: List<SuggestedControl<ID>> = neighborsVel.neighbors.list.map { SuggestedControl(Edge(localId, it.id), robot.velocity to it.value) }
////    val residuals: List<Residual<ID>> = neighborsVel.neighbors.ids.list.map { Residual(Edge(localId, it), 0.0, 0.0) }
//    return Parameters(robot, coupledEdges)
//}

/**
 * Edge owner policy where the device with the smallest [ID] is the owner.
 * Where [id] is the neighbor [ID] to compare.
 */
private fun <ID: Comparable<ID>> Aggregate<ID>.isOwner(id: ID): Boolean = min(localId, id) == localId

fun updateResiduals(): Unit = TODO("of all pairs")

fun <LocalResidual> updateLocalResidual(): LocalResidual = TODO()

fun updateDualResidual(res: Double): LocalUpdate = TODO("to the neighbors")

fun updateLocalControl(): Unit = TODO("given by the overall iterations of ADMM")

fun localResidualThreshold(): Boolean = TODO()

fun applyControl(): Unit = TODO("apply control to current robot")

//fun Aggregate<Int>.fullADMM(
//    position: LocationSensor,
//    env: EnvironmentVariables,
//    device: CollektiveDevice<Euclidean2DPosition>,
//): Unit = context(position, device) {
//    val confidenceThreshold = 50
//    val target: Target = getTarget(env["TargetID"] as Number)
//    val tolerance = Tolerance(1e-3, 1e-3) // used the same tolerance as in ADMM Example3
//    val robot = getRobot(localId)
//    val neighborsPositions = neighboring(robot.position)
//    val localParams = initParameters(robot)
//    evolve(localParams) { params ->
//        for (iteration in 0 until confidenceThreshold) { // todo this cycle breaks aggregate
//            executeCoreADMM(robot, target, params)
//            // stop if got to residual threshold, no need to execute all iterations
//            if (localResidualThreshold()) break
//        }
//        updateLocalControl()
//        params // should be the updated ones
//    }
//    applyControl()
//}
//
//context(position: LocationSensor, device: CollektiveDevice<Euclidean2DPosition>)
//fun Aggregate<Int>.executeCoreADMM(robot: Robot<Int>, target: Target, params: Parameters<Int>) {
//    val (uWanted, deltaNom) = avoidObstacleGoToTarget(robot, target, getObstacle(), params)
////    val uNeighbors = neighboring(uWanted).neighbors.list
//    val updatedRobot = robot.copy(control = uWanted)
//    val neighbors = neighboring(updatedRobot).neighbors.list
//
//    if (neighbors.isNotEmpty()) {
//        neighbors.forEach { n ->
//            if (isOwner(n.id)) {
//                val edge = params.edges.find { it.neighbor == n.id }!!
//                val suggested = robotAvoidanceAndCommunicationRangeCBF(updatedRobot, n.value, 10.0, edge)
//                // todo and share them
//                moveNodeToPosition(robot.position + suggested.controlForLocal)
//                moveNodeToPosition(edge.neighbor, n.value.position + suggested.controlForOther)
//            } else {
//                // receive update desired speed from owner
//            }
//            updateResiduals()
//        }
//    }
//    val localResidualsUpdated = updateLocalResidual<Double>()
//    updateDualResidual(localResidualsUpdated)
//}
