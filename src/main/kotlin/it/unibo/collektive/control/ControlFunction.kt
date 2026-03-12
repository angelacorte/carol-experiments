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

    val slack: GRBVar?

    val slackWeight: Double?

    fun add(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext)

    fun addSlackToObjective(obj: GRBExpr) {
        slack ?: return
        when(obj) {
            is GRBLinExpr -> obj.addTerm(slackWeight ?: 1.0, slack)
            is GRBQuadExpr -> obj.addTerm(slackWeight ?: 1.0, slack, slack)
            else -> error("cannot add slack to objective")
        }
    }
}

/**
 * Context for CF builders carrying [self], optional [otherRobot], and solver [settings].
 */
data class ControlFunctionContext(
    val self: Robot,
    val otherRobot: Robot? = null,
    val settings: QpSettings = QpSettings(),
)
