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

/**
 * Communication-range barrier;
 * enforces a maximum connection distance [range] between two robots.
 * When [slackWeight] is provided, creates a slack variable to soften the constraint;
 * otherwise the constraint is hard.
 *
 * Implements the robustified discrete-time CBF (ZOH dynamics).
 */
class CommunicationRangeCBF(
    private val range: Double,
    override val eta: Double = 0.5,
    override val slackWeight: Double? = null,
) : CBF() {
    override val name: String = "comm_range"

    override var slack: GRBVar? = null

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext) {
        check(uOther != null && context.otherRobot != null) {
            "Cannot apply Communication CBF with other robot as null value, $uOther"
        }
        val distance = (context.self.position - context.otherRobot.position).toDoubleArray()
        val h = range.pow(2) - distance.squaredNorm()
        val uMax = max(context.self.maxSpeed, context.otherRobot.maxSpeed)
        val dt = context.settings.deltaTime
        // RHS: -(\eta / \Delta t) * h_{ij,k}^{com} + 4 * \Delta t * u_{max}^2
        // The factor 4 is required to conservatively lower bound the exact ZOH expansion
        val rhs = -(eta / dt) * h + 4.0 * dt * uMax.pow(2)
        // LHS: -2 * r_{ij}^T * (u_i - u_j)
        val lhs = GRBLinExpr()
        for (index in distance.indices) {
            lhs.addTerm(-2.0 * distance[index], uSelf[index])
            lhs.addTerm(2.0 * distance[index], uOther[index])
        }
        if (slackWeight != null) {
            val s = addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, ConstraintNames.slack(name))
            slack = s
            lhs.addTerm(1.0, s)
        }
        addConstr(lhs, GRB.GREATER_EQUAL, rhs, ConstraintNames.comm("${context.self.position}_${context.otherRobot.position}"))
    }
}
