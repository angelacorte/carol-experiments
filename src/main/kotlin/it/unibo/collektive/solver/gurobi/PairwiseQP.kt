package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.admm.LocalDualUpdate
import it.unibo.collektive.admm.SuggestedControl
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.mathutils.plus
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D

class PairwiseQP private constructor(
    private val model: GRBModel,
    private val slack: GRBVar,
    private val zi: GRBVector,
    private val zj: GRBVector,
    private val constraints: List<Constraint>,
) {

    fun dispose() = model.dispose()

    fun updateAndSolve(
        robot: Robot,
        other: Robot,
        incidentDuals: LocalDualUpdate,
        settings: QpSettings,
        deltaTime: Double,
    ): SuggestedControl {
        val robotArray = robot.control.toDoubleArray()
        val otherArray = other.control.toDoubleArray()
        for (i in zi.variables.indices) {
            zi[i].set(GRB.DoubleAttr.LB, -robot.maxSpeed)
            zi[i].set(GRB.DoubleAttr.UB, robot.maxSpeed)
            zi[i].set(GRB.DoubleAttr.Start, robotArray[i]) // warm start
            zj[i].set(GRB.DoubleAttr.LB, -other.maxSpeed)
            zj[i].set(GRB.DoubleAttr.UB, other.maxSpeed)
            zj[i].set(GRB.DoubleAttr.Start, otherArray[i]) // warm start
        }
        constraints.forEach { constraint -> constraint.update(model, robot, other, settings, deltaTime) }
        model.setObjective(buildObjective(robot, other, incidentDuals, settings), GRB.MINIMIZE)
        model.update()
        model.optimize()
        return extractSolution(robot, other)
    }

    private fun buildObjective(
        robot: Robot,
        other: Robot,
        incidentDuals: LocalDualUpdate,
        settings: QpSettings,
    ): GRBQuadExpr = GRBQuadExpr().apply {
        val rho = settings.rhoADMM / 2.0
        // (ρ/2)‖z_i − (u_i + y_i)‖²  +  (ρ/2)‖z_j − (u_j + y_j)‖²
        addRhoNorm2Sq(zi, (robot.control + incidentDuals.yi).toDoubleArray(), rho)
        addRhoNorm2Sq(zj, (other.control + incidentDuals.yj).toDoubleArray(), rho)
        constraints.forEach { constr ->
            constr.slack?.let { slackC ->
                val weight = constr.slackWeight ?: settings.rhoSlack
                addTerm(weight, slackC, slackC)
            }
        }
        addTerm(settings.rhoSlack, slack, slack)
    }

    private fun extractSolution(robot: Robot, other: Robot): SuggestedControl {
        val status = model.get(GRB.IntAttr.Status)
        if (status == GRB.INFEASIBLE) {
            model.writeIIS("commonModel.ilp")
            for (c in model.constrs) {
                if (c.get(GRB.IntAttr.IISConstr) == 1) {
                    println("Pairwise IIS constraint: ${c.get(GRB.StringAttr.ConstrName)}")
                }
            }
        }
        return when {
            model.get(GRB.IntAttr.SolCount) > 0 -> SuggestedControl(
                SpeedControl2D(zi[0].get(GRB.DoubleAttr.X), zi[1].get(GRB.DoubleAttr.X)),
                SpeedControl2D(zj[0].get(GRB.DoubleAttr.X), zj[1].get(GRB.DoubleAttr.X)),
            )
            else -> {
                println("Pairwise QP: no solution found (status $status), returning current controls.")
                SuggestedControl(robot.control, other.control)
            }
        }
    }

    companion object {

        fun create(model: GRBModel, robot: Robot, other: Robot, pairwiseCBFs: List<CBF>): PairwiseQP {
            val slack = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_pairwiseQP")
            val zi = model.addVecVar(robot.position.dimension, -robot.maxSpeed, robot.maxSpeed, "z_ij^i")
            val zj = model.addVecVar(other.position.dimension, -other.maxSpeed, other.maxSpeed, "z_ij^j")
            val constrs = mutableListOf<Constraint>()
            pairwiseCBFs.forEach { cbf -> constrs += cbf.install(model, zi, zj) }
            model.update()
            return PairwiseQP(
                model = model,
                slack,
                zi = zi,
                zj = zj,
                constraints = constrs,
            )
        }
    }
}
