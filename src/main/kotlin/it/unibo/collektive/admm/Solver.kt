package it.unibo.collektive.admm

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBModel
import it.unibo.alchemist.core.Simulation
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.model.Robot
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.solver.gurobi.LocalQP
import it.unibo.collektive.solver.gurobi.PairwiseQP
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.setLicense
import it.unibo.collektive.solver.gurobi.setupLogger
import java.util.concurrent.ConcurrentHashMap

class Solver(val settings: QpSettings) {

    private lateinit var local: LocalQP

    private lateinit var pairwise: PairwiseQP

    private val env: GRBEnv = setLicense().let {
        GRBEnv(true).also {
            it.set(GRB.IntParam.OutputFlag, if (settings.logEnabled) 1 else 0)
            it.start()
        }
    }

    fun dispose() {
        local.model.dispose()
        pairwise.model.dispose()
        env.dispose()
    }

    fun setupLocalModel(
        robot: Robot,
        localCLFs: List<CLF>,
        localCBFs: List<CBF>,
    ) {
        if (!this::local.isInitialized) {
            val model = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }
            local = LocalQP.create(model, robot, localCLFs, localCBFs)
        }
    }

    fun setupPairwiseModel(
        robot: Robot,
        otherRobot: Robot,
        pairwiseCBFs: List<CBF>,
    ) {
        if (!this::pairwise.isInitialized) {
            val model = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }
            pairwise = PairwiseQP.create(model, robot, otherRobot, pairwiseCBFs)
        }
    }

    fun <ID: Comparable<ID>> updateAndSolveLocal(
        robot: Robot,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        context: ControlFunctionContext,
    ): SpeedControl2D = local.updateAndSolve(robot, uNominal, duals, context)

    fun <ID : Comparable<ID>> updateAndSolvePairwise(
        robot: Robot,
        otherRobot: Robot,
        duals: IncidentDuals,
        context: ControlFunctionContext,
    ): SuggestedControl = pairwise.solve(robot, otherRobot, duals, context)
}

object SimulationSolver {
    private val activeSolvers: MutableMap<Simulation<*, *>, Solver> = ConcurrentHashMap()

    private fun get(): Solver {
        TODO("return solver associated with simulation")
    }

    val localSolver: LocalQP
        get() = get().local


    val pairwiseSolver: PairwiseQP
        get() = get().pairwise

}
