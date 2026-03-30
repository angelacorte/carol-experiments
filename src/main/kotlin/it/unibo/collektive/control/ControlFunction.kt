package it.unibo.collektive.control

import com.gurobi.gurobi.GRBExpr
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings

/**
 * A control function (CLF or CBF) that integrates into an ADMM QP via a two-phase protocol:
 *
 * [install] adds all decision variables and constraints to a fresh [GRBModel] and returns an
 * [Constraint] that owns the resulting GRB handles.  After this call the model
 * **topology** is fixed: no further `addVar` / `addConstr` calls will be made for this function.
 *
 * [Constraint.update] rewrites only the *numerical* parameters — RHS values and linear
 * coefficients — via [GRBModel.chgCoeff] and attribute setters.  A single [GRBModel.update] call
 * is issued by the owning template after all constraints have been refreshed.
 */
interface ControlFunction {

    /** Unique identifier used for Gurobi constraint naming. */
    val name: String

    /**
     * Penalty weight applied to the slack variable in the QP objective, or `null` when this control
     * function should not introduce/use its own slack.
     */
    val slackWeight: Double?

    fun install(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?): Constraint

    /**
     * Legacy helper for objective slack penalties.
     * The active QP builders currently manage slack penalties directly.
     */
    fun addSlackToObjective(objective: GRBExpr, slack: GRBVar, settings: QpSettings) {
        val weight = slackWeight ?: settings.rhoSlack
        when (objective) {
            is GRBLinExpr -> objective.addTerm(weight, slack)
            is GRBQuadExpr -> objective.addTerm(weight, slack)
            else -> error("Cannot add slack to objective of type ${objective::class.simpleName}")
        }
    }
}
