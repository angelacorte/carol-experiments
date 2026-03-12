package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.CFContext
import it.unibo.collektive.model.Target
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.squaredNorm
import it.unibo.collektive.model.toDoubleArray
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.createAndAddSlack
import it.unibo.collektive.solver.gurobi.toLinExpr
import kotlin.math.pow

class GoToTargetCLF(val target: Target) : CLF() {
    override val name: String = "go_to_target"

    override var slack: GRBVar? = null

    /**
     * Discrete-time CLF (DCLF) constraint for goal reaching under ZOH dynamics.
     *
     * From the decrease template V_{k+1} ≤ (1−λ)V_k + δ with V_k = ‖e_k‖²
     * and the upper bound ‖u_k‖² ≤ u²_max, the affine sufficient constraint is:
     *
     *     2∆t eᵀ u − δ  ≤  −λ ‖e‖²  −  ∆t² u²_max
     *
     * where e = p − p_g, λ = [it.unibo.collektive.solver.gurobi.QpSettings.convergenceRate], ∆t = [it.unibo.collektive.solver.gurobi.QpSettings.deltaTime].
     */
    override fun GRBModel.applyCLF(uSelf: GRBVector, context: CFContext) {
        require(context.settings.deltaTime.isFinite() && context.settings.deltaTime > 0.0) {
            "deltaTime must be finite and greater than zero to build DCLF constraint"
        }
        require(context.self.maxSpeed.isFinite() && context.self.maxSpeed >= 0.0) { "maxSpeed must be finite and non-negative" }
        require(target.x.isFinite() && target.y.isFinite()) { "Target coordinates must be finite" }
        val distanceVec = (context.self.position - target.position).toDoubleArray()
        val dt = context.settings.deltaTime
        // left side: 2∆t eᵀ u − δ
        val left = uSelf.toLinExpr(distanceVec, 2.0 * dt)
        slack = createAndAddSlack(left, slackName = ConstraintNames.slack(name), -1.0)
        // right side: −λ ‖e‖² − ∆t² u²_max
        val right = -context.settings.convergenceRate * distanceVec.squaredNorm() - dt.pow(2) * context.self.maxSpeed.pow(2)
        addConstr(left, GRB.LESS_EQUAL, right, ConstraintNames.clf(target.id.toString()))
        addConstr(slack, GRB.GREATER_EQUAL, 0.0, ConstraintNames.slack(target.id.toString()))
    }
}
