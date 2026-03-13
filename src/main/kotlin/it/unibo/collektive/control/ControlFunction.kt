package it.unibo.collektive.control

import com.gurobi.gurobi.GRBExpr
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.model.Robot
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings

interface ControlFunction {
    val name: String
    val slackWeight: Double?

    /** Applies the constraint to the model and returns the generated slack variable (if any). */
    fun applyToModel(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext): GRBVar?

    /** Adds the linear penalty for the generated slack variable to the objective. */
    fun addSlackToObjective(obj: GRBExpr, slack: GRBVar, context: ControlFunctionContext) {
        val weight = slackWeight ?: context.settings.rhoSlack
        when (obj) {
            is GRBLinExpr -> obj.addTerm(weight, slack)
            is GRBQuadExpr -> obj.addTerm(weight, slack)
            else -> error("Cannot add slack to objective of type ${obj::class.simpleName}")
        }
    }
}

data class ControlFunctionContext(
    val self: Robot,
    val otherRobot: Robot? = null,
    val settings: QpSettings = QpSettings(),
)
