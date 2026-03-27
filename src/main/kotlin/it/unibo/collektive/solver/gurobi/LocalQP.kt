package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.admm.DualParams
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D

class LocalQP private constructor(
    private val model: GRBModel,
    private val u: GRBVector,
    private val slack: GRBVar,
    private val constraints: List<Constraint>,
) {

    fun dispose() = model.dispose()

    fun <ID : Comparable<ID>> updateAndSolve(
        robot: Robot,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        settings: QpSettings,
        deltaTime: Double,
    ): SpeedControl2D {
        for (i in u.variables.indices) {
            u[i].set(GRB.DoubleAttr.LB, -robot.maxSpeed)
            u[i].set(GRB.DoubleAttr.UB, robot.maxSpeed)
            u[i].set(GRB.DoubleAttr.Start, robot.control.toDoubleArray()[i]) // warm start
        }
        constraints.forEach { constraint ->
            constraint.update(model, robot, settings = settings, deltaTime = deltaTime)
        }
        model.setObjective(buildObjective(uNominal, duals, settings), GRB.MINIMIZE)
        model.update()
        model.optimize()
        return extractSolution(robot)
    }

    private fun buildObjective( // todo this should be taken from outside, can be different by different simulations
        uNominal: DoubleArray,
        duals: Map<*, DualParams>,
        settings: QpSettings,
    ): GRBQuadExpr = GRBQuadExpr().apply {
        addRhoNorm2Sq(u, uNominal)
        constraints.forEach { constr ->
            constr.slack?.let { slackC ->
                val weight = constr.slackWeight ?: settings.rhoSlack
                addTerm(settings.rhoSlack, slackC, slackC)
            }
        }
        addTerm(settings.rhoSlack, slack, slack)
        duals.forEach { (_, value) ->
            val suggested = value.suggestedControl.zi.toDoubleArray()
            val residual = value.incidentDuals.yi.toDoubleArray()
            addRhoNorm2Sq(u, suggested - residual, settings.rhoADMM / 2.0)
        }
    }

    private fun extractSolution(robot: Robot): SpeedControl2D {
        val status = model.get(GRB.IntAttr.Status)
        if (status == GRB.INFEASIBLE) {
            model.writeIIS("localModel.ilp")
            for (constr in model.constrs) {
                if (constr.get(GRB.IntAttr.IISConstr) == 1) {
                    println("Local IIS constraint: ${constr.get(GRB.StringAttr.ConstrName)}")
                }
            }
        }
        return when {
            model.get(
                GRB.IntAttr.SolCount,
            ) > 0 -> SpeedControl2D(u[0].get(GRB.DoubleAttr.X), u[1].get(GRB.DoubleAttr.X))
            else -> {
                println("Local QP: no solution found (status $status), returning previous control.")
                robot.control
            }
        }
    }

    companion object {
        fun create(model: GRBModel, robot: Robot, localCLFs: List<CLF>, localCBFs: List<CBF>): LocalQP {
            val u = model.addVecVar(robot.position.dimension, -robot.maxSpeed, robot.maxSpeed, "u")
            val slack = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_localQP")
            val installed = mutableListOf<Constraint>()
            localCLFs.forEach { clf -> installed += clf.install(model, u, null) }
            localCBFs.forEach { cbf -> installed += cbf.install(model, u, null) }
            model.update()
            return LocalQP(
                model = model,
                u = u,
                slack,
                constraints = installed,
            )
        }
    }
}
