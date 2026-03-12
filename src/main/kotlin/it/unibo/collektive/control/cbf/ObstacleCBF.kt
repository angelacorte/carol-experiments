package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.model.Obstacle
import it.unibo.collektive.model.minus
import it.unibo.collektive.model.squaredNorm
import it.unibo.collektive.model.toDoubleArray
import it.unibo.collektive.solver.gurobi.ConstraintNames
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.createAndAddSlack
import it.unibo.collektive.solver.gurobi.toLinExpr
import kotlin.math.pow

/**
 * Obstacle-avoidance barrier;
 * adds keep-out CBF against obstacle.
 * When [withSlack] is `true`, creates a slack variable to soften the constraint;
 * otherwise the constraint is hard.
 *
 * @param withSlack whether to add a slack variable to relax the constraint.
 * @param slackWeight penalty weight for the slack variable (default: 0.0)
 */
class ObstacleCBF(
    val obstacle: Obstacle,
) : CBF {
    override val name: String = "obstacle_avoidance"

    override fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: CFContext) {
        val distance = (context.self.position - obstacle).toDoubleArray()
        val safeDistance = obstacle.radius + obstacle.margin
        val h = distance.squaredNorm() - safeDistance.pow(2)
        val expr = uSelf.toLinExpr(distance, 2.0)
        val rhs = -context.settings.gammaObstacle / context.settings.deltaTime * h
        addConstr(expr, GRB.GREATER_EQUAL, rhs, ConstraintNames.obstacle("local"))
    }
}
