package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.squaredNorm
import it.unibo.collektive.model.toDoubleArray
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import kotlin.math.max
import kotlin.math.pow
import org.apache.commons.math3.special.Gamma

/**
 * Robot–robot collision avoidance barrier;
 * enforces separation from the context's other robot under ZOH dynamics.
 */
class CollisionAvoidanceCBF(override val eta: Double = 0.5, override val slackWeight: Double? = null) : CBF() {
    override val name: String = "collision_avoidance"
    override var slack: GRBVar? = null

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext) {
        check(context.otherRobot != null && uOther != null) {
            "Other robot must not be null to apply Collision Avoidance CBF"
        }
        val distance = (context.self.position - context.otherRobot.position).toDoubleArray()
        val maxDist = max(context.self.safeMargin, context.otherRobot.safeMargin)
        val h = distance.squaredNorm() - maxDist.pow(2)
        val dt = context.settings.deltaTime
        // RHS: -(\eta / \Delta t) * h_{ij,k}^{col}
        val rhs = -(eta / dt) * h
        // LHS: 2 * r_{ij}^T * (u_i - u_j)
        val lhs = GRBLinExpr()
        for (index in distance.indices) {
            lhs.addTerm(2.0 * distance[index], uSelf[index])
            lhs.addTerm(-2.0 * distance[index], uOther[index])
        }
        // Soften the constraint if a slack weight was specified
        if (slackWeight != null) {
            val s = addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, ConstraintNames.slack(name))
            slack = s
            lhs.addTerm(1.0, s)
        }
        addConstr(lhs, GRB.GREATER_EQUAL, rhs, ConstraintNames.collision("${context.self.position}_${context.otherRobot.position}"))
    }
}
