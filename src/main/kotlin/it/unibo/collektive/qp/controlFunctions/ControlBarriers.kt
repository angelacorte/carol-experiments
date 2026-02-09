package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.qp.Coupled
import it.unibo.collektive.qp.SuggestedControl
import it.unibo.collektive.qp.dsl.GRBVector
import it.unibo.collektive.qp.dsl.addCBF
import it.unibo.collektive.qp.dsl.minus
import it.unibo.collektive.qp.dsl.toQuadExpr
import it.unibo.collektive.qp.dsl.squaredNorm
import it.unibo.collektive.qp.dsl.zeroVec
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot
import it.unibo.collektive.qp.utils.SpeedControl2D
import it.unibo.collektive.qp.utils.Target
import it.unibo.collektive.qp.utils.minus
import it.unibo.collektive.qp.utils.toDoubleArray
import kotlin.math.max
import kotlin.math.pow

/**
 * (OBSTACLE AVOIDANCE) linear CBF 2(p - p_o)^T u >= - \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ]
 */
fun GRBModel.addObstacleAvoidanceCBF(
    currentPosition: DoubleArray,
    obstacle: Obstacle,
    u: GRBVector,
    gamma: Double = 0.5, // \gamma in {0.5 .. 5} = soft || in {5, 20} = hard || > infeasible QP
) {
    // 2(p - p-g)^T u = 2(p_x - p_o,x) u_x + 2(p_y - p_o,y) u_y
    val obstaclePosition: DoubleArray = obstacle.toDoubleArray()
    val distance = currentPosition - obstaclePosition // ||p - p_o||^2
    val safeDistance = obstacle.radius + obstacle.margin
    // - \gamma [ ||p - p_o||^2 - (r_o + d_o)^2 ] ==== - \gamma ((p_x - p_o,x)^2 + (p_y - p_o,y) ^2 - (r_o + d_o)^2)
    val h = distance.squaredNorm() - safeDistance.pow(2)
    addCBF(
        p1 = currentPosition,
        p2 = obstaclePosition,
        u1 = u,
        u2 = zeroVec(u.dimensions),
        gamma = gamma,
        h = h, // ( (p_x - p_o,x)^2 + (p_y - p_o,y) ^2 - (r_o^2 + d_o^2)
        name = "obstacleAvoidance",
        coefU1 = 2.0,
        coefU2 = 0.0,
    )
}

fun <ID: Comparable<ID>> GRBModel.addCollisionAvoidanceCBF(ui: GRBVector, uj: GRBVector, robot: Robot<ID>, other: Robot<ID>) {
    // COLLISION AVOIDANCE 2(p1 - p2)^T (u1 - u2) + \gamma [ ||p1-p2||^2 - dmin^2 ] >= 0
    // 2(p1 - p2)^T (u1 - u2) >= - \gamma [ ||p1-p2||^2 - dmin^2 ]
    val gamma = 0.5
    val distance = (robot.position - other.position).toDoubleArray()
    val maxDist = max(robot.safeMargin, other.safeMargin)
    val collision = GRBLinExpr()
    val collRight = -gamma * (distance.squaredNorm() - maxDist.pow(2))
    for (index in 0 until distance.size) {
        collision.addTerm(2.0 * distance[index], ui[index])
        collision.addTerm(-2.0 * distance[index], uj[index])
    }
    addConstr(collision, GRB.GREATER_EQUAL, collRight, "collision_avoidance_${robot.id}VS${other.id}")
}

fun <ID: Comparable<ID>> GRBModel.addCommunicationRangeCBF(ui: GRBVector, uj: GRBVector, robot: Robot<ID>, other: Robot<ID>, range: Double) {
    // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
    // COMM DISTANCE -2(p1 - p2)^T (u1 -u2) >= - \gamma [ R^2 - ||p1 - p2||^2 ]
    val gamma = 0.5
    val distance = (robot.position - other.position).toDoubleArray()
    val communication = GRBLinExpr()
    val commRight = -gamma * (range.pow(2) - distance.squaredNorm())
    for (index in 0 until distance.size) {
        communication.addTerm(-2.0 * distance[index], ui[index])
        communication.addTerm(2.0 * distance[index], uj[index])
    }
    addConstr(communication, GRB.GREATER_EQUAL, commRight, "communication_range_${robot.id}VS${other.id}")
}

// || u - u_nom||^2 + rho_s * delta^2 + rho_a / 2 * SUM ||i - z_ij,i + y_ij,i||^2
fun <ID: Comparable<ID>> GRBModel.minimizeADMMLocalQP(u: GRBVector, delta: GRBVar, robot: Robot<ID>, target: Target, edges: List<Coupled<ID>>): Pair<SpeedControl2D, Double> {
    val rhoSlack = 2.0
    val rhoADMM = 10.0
    val uNominal = (robot.position - target.position).toDoubleArray()
    val obj = GRBQuadExpr()
    // ||u - u_nom||^2
    for (k in 0 until robot.dimension) {
        obj.addTerm(1.0, u[k], u[k])
        obj.addTerm(-2.0 * uNominal[k], u[k])
    }
    // slack
    obj.addTerm(rhoSlack, delta, delta)
    val edgesCount = edges.size
    val sumCoeff = rhoADMM * edgesCount / 2.0
    // ADMM proximal term
    edges.forEach { couple ->
        val suggestedControl = couple.suggestedControl.controlForLocal.toDoubleArray()
        val residual = couple.residuals.valueForLocal.toDoubleArray()
        for (k in 0 until robot.dimension) {
            obj.addTerm(sumCoeff, u[k], u[k])
            obj.addTerm(-1.0 * suggestedControl[k], u[k])
            obj.addTerm(residual[k], u[k])
        }
    }
    setObjective(obj, GRB.MINIMIZE)
    // solve
    optimize()
    val uOptX = u[0].get(GRB.DoubleAttr.X)
    val uOptY = u[1].get(GRB.DoubleAttr.X)
    val deltaOpt = delta.get(GRB.DoubleAttr.X)
    println("Optimal control for ${robot.id}: u = ($uOptX, $uOptY)")
    return SpeedControl2D(uOptX, uOptY) to deltaOpt
}

// rho / 2 * ( ||z_ij,i - (ui + y_ij,i)||^2 + || z_ij,j - (uj + y_ij,j)||^2 )
fun <ID: Comparable<ID>> GRBModel.minimizeADMMCommonQP(zi: GRBVector, zj: GRBVector, robot: Robot<ID>, other: Robot<ID>, edge: Coupled<ID>): SuggestedControl<ID> {
    val rhoADMM = 10.0
    val obj = GRBQuadExpr()
    val ui = robot.velocity.toDoubleArray()
    val uj = other.velocity.toDoubleArray()
    val yi = edge.residuals.valueForLocal.toDoubleArray()
    val yj = edge.residuals.valueForOther.toDoubleArray()
    // ||zij,i - (ui + yij,i)||^2
    for (k in 0 until robot.dimension) {
        obj.addTerm(1.0, zi[k], zi[k])
        obj.addTerm(- 1.0  * ui[k], zi[k])
        obj.addTerm(1.0 * yi[k], zi[k])
    }
    // + ||zij,j - (uj + yij,j)||^2
    for (k in 0 until robot.dimension) {
        obj.addTerm(1.0, zj[k], zj[k])
        obj.addTerm(- 1.0  * uj[k], zj[k])
        obj.addTerm(1.0 * yj[k], zj[k])
    }
    setObjective(obj, GRB.MINIMIZE)
    // solve
    optimize()
    val zxiOpt = zi[0].get(GRB.DoubleAttr.X)
    val zyiOpt = zi[1].get(GRB.DoubleAttr.X)
    val zxjOpt = zi[0].get(GRB.DoubleAttr.X)
    val zyjOpt = zi[1].get(GRB.DoubleAttr.X)
    println("Optimal control for ${robot.id}: u = ($zxiOpt, $zyiOpt)")
    println("Optimal control for ${other.id}: u = ($zxjOpt, $zyjOpt)")
    return SuggestedControl(SpeedControl2D(zxiOpt, zyiOpt), SpeedControl2D(zxjOpt, zyjOpt))
}



//
///**
// * (COMMUNICATION DISTANCE) CBF -2(p1 - p2)^T (u1 -u2) + \gamma [ R^2 - ||p1 - p2||^2 ] >= 0
// * move to the right
// * -2(p1 - p2)^T (u1 -u2) >= - \gamma [ R^2 - ||p1 - p2||^2 ]
// * -2(p1 - p2)^T u1 + 2(p1 - p2)^T u2 >= - \gamma [ R^2 - ||p1 - p2||^2 ]
// * -2(p1 - p2)^T u1 >= -2(p1 - p2)^T u2 - \gamma [ R^2 - ||p1 - p2||^2 ]
// * ||p1 - p2||^2 = (p1 - p2)^T (p1 -p2)
// */
//fun <ID: Comparable<ID>> GRBModel.addCommunicationRangeCBF(
//    maxConnectionDistance: Double,
//    robotsToBeConnected: List<Robot<ID>>,
//    position: DoubleArray,
//    u: GRBVector,
//    robot: Robot<ID>,
//) {
//    val maxDistSq = maxConnectionDistance.pow(2) // R^2
//    val gamma = 0.5
//    robotsToBeConnected.forEach { connect ->
//        val positionOther: DoubleArray = connect.toDoubleArray()
//        val velocityOther: DoubleArray = connect.velocity.toDoubleArray()
//        // 2(p1-p2)^T u2 - \gamma [ R^2 - (p1-p2)^T(p1-p2)  ]
//        // (p1-p2)^T(p1-p2) = (p1x - p2x) p1x + (p1y - p2y) p1y
//        // (p1x - p2x) p1x = dxr * uxa
//        // (p1y - p2y) p1y = dyr * uya
//        val h = maxDistSq - (position - positionOther).squaredNorm()
//        addCBF(
//            p1 = positionOther,
//            p2 = position,
//            u1 = u,
//            u2 = velocityOther,
//            gamma = gamma,
//            h = h,
//            name = "communicationRange_${robot.id}_with_${connect.id}",
//            coefU1 = -2.0,
//            coefU2 = -2.0,
//        )
//    }
//}

///**
// * (ROBOT AVOIDANCE) linear CBF 2(p1 - p2)^T (u1 - u2) + \gamma [ ||p1-p2||^2 - dmin^2 ] >= 0
// * 2(p1 - p2)^T (u1 - u2) + \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ] >= 0
// * move u2 to the right
// * 2(p1 - p2)^T u1 -2 (p1 - p2)^T u2 >= - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
// * 2(p1 - p2)^T u1 >= 2(p1 - p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
// */
//fun <ID: Comparable<ID>> GRBModel.addRobotAvoidanceCBF(
//    robotsToAvoid: List<Robot<ID>>,
//    robot: Robot<ID>,
//    position: DoubleArray,
//    u: GRBVector,
//) = robotsToAvoid.forEach { avoid ->
//    val positionOther: DoubleArray = avoid.toDoubleArray()
//    val velocityOther: DoubleArray = avoid.velocity.toDoubleArray()
//    val minDistSq = max(robot.safeMargin, avoid.safeMargin).pow(2)
//    val gamma = 0.5 // \gamma in {0.5 .. 5} = soft || in {5, 20} = hard || > infeasible QP
//    // (p1 - p2)^T (p1 - p2) = (p1x - p2x)^2 + (p1y - p2y)^2
//    val h = (position - positionOther).squaredNorm() - minDistSq
//    // left side
//    // 2(p1-p2)^T u1 // my velocity
//    // right side
//    // -2(p1-p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
//    addCBF(
//        p1 = position,
//        p2 = positionOther,
//        u1 = u,
//        u2 = velocityOther,
//        gamma = gamma,
//        h = h, // -2(p1-p2)^T u2 - \gamma [ (p1-p2)^T(p1-p2) - dmin^2 ]
//        name = "robotAvoidance_${robot.id}_vs_${avoid.id}",
//        coefU1 = 2.0,
//        coefU2 = 2.0,
//    )
//}

/**
 * norm constraint on the control input ux^2 + uy^2 <= maxSpeed^2
 */
fun <ID: Comparable<ID>> GRBModel.maxSpeedCBF(
    u: GRBVector,
    robot: Robot<ID>,
) {
    addQConstr(
        u.toQuadExpr(),
        GRB.LESS_EQUAL,
        robot.maxSpeed.pow(2),
        "u_norm"
    )
}
