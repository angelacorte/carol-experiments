package it.unibo.collektive.alchemist.device.sensors.impl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBModel
import it.unibo.alchemist.boundary.OutputMonitor
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.NodeProperty
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.Time
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.collektive.admm.DualParams
import it.unibo.collektive.admm.LocalDualUpdate
import it.unibo.collektive.admm.SuggestedControl
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.model.Device
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.solver.Solver
import it.unibo.collektive.solver.gurobi.LocalQP
import it.unibo.collektive.solver.gurobi.PairwiseQP
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.setLicense
import it.unibo.collektive.solver.gurobi.setupLogger

/**
 * High-level facade that owns the reusable Gurobi environments and QP templates used by ADMM.
 *
 * A solver lazily builds one local model and one pairwise model, then updates and solves them at
 * each step. The single pairwise template is shared by every edge of the node: only numerical
 * coefficients are refreshed per edge, which assumes all nodes install structurally identical
 * pairwise CBFs (currently true for every scenario). Rebuilding a Gurobi model per edge or per
 * step would dominate the control period, so the templates are reused for the whole simulation
 * and disposed together with the [GRBEnv] when the simulation finishes.
 *
 * Solves that end without a usable solution fall back to the previous control and are counted in
 * the [LOCAL_QP_NO_SOLUTION_MOLECULE] and [PAIRWISE_QP_NO_SOLUTION_MOLECULE] molecules, so exports
 * can tell clean runs from degraded ones.
 *
 * @property settings numerical and logging configuration shared by every managed QP.
 */
class SolverProperty<T, P : Position<P>>(override val settings: QpSettings, override val node: Node<T>) :
    Solver,
    NodeProperty<T> {

    private lateinit var local: LocalQP

    private lateinit var pairwise: PairwiseQP

    private var localNoSolutionCount: Long = 0

    private var pairwiseNoSolutionCount: Long = 0

    private val env: GRBEnv = setLicense().let {
        GRBEnv(true).also { env ->
            env.set(GRB.IntParam.OutputFlag, if (settings.logEnabled) 1 else 0)
            env.start()
        }
    }

    init {
        recordCount(LOCAL_QP_NO_SOLUTION_MOLECULE, 0)
        recordCount(PAIRWISE_QP_NO_SOLUTION_MOLECULE, 0)
    }

    override val isLocalModelAvailable: Boolean get() = this::local.isInitialized

    override val isPairwiseModelAvailable: Boolean get() = this::pairwise.isInitialized

    override fun setupLocalModel(device: Device, localCLFs: List<CLF>, localCBFs: List<CBF>) {
        if (!isLocalModelAvailable) {
            val model = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }
            local = LocalQP.create(model, device, localCLFs, localCBFs) { _ ->
                recordCount(LOCAL_QP_NO_SOLUTION_MOLECULE, ++localNoSolutionCount)
            }
        } else {
            local.syncControlFunctions(localCLFs, localCBFs)
        }
    }

    override fun setupPairwiseModel(device: Device, otherDevice: Device, pairwiseCBFs: List<CBF>) {
        if (!isPairwiseModelAvailable) {
            val model = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }
            pairwise = PairwiseQP.create(model, device, otherDevice, pairwiseCBFs) { _ ->
                recordCount(PAIRWISE_QP_NO_SOLUTION_MOLECULE, ++pairwiseNoSolutionCount)
            }
        } else {
            pairwise.syncControlFunctions(pairwiseCBFs)
        }
    }

    override fun <ID : Comparable<ID>> updateAndSolveLocal(
        device: Device,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        deltaTime: Double,
    ): SpeedControl2D = local.updateAndSolve(device, uNominal, duals, settings, deltaTime)

    override fun updateAndSolvePairwise(
        device: Device,
        otherDevice: Device,
        duals: LocalDualUpdate,
        deltaTime: Double,
    ): SuggestedControl = pairwise.updateAndSolve(device, otherDevice, duals, settings, deltaTime)

    override fun cloneOnNewNode(node: Node<T>): NodeProperty<T> = SolverProperty(settings, node)

    private fun recordCount(molecule: SimpleMolecule, count: Long) {
        @Suppress("UNCHECKED_CAST")
        node.setConcentration(molecule, count as T)
    }

//    private fun disposeAll() {
//        if (isLocalModelAvailable) local.dispose()
//        if (isPairwiseModelAvailable) pairwise.dispose()
//        env.dispose()
//    }

    /**
     * Names of the molecules exporting solver health counters.
     */
    companion object {
        /**
         * Cumulative count of local QP solves that ended without a solution.
         */
        val LOCAL_QP_NO_SOLUTION_MOLECULE = SimpleMolecule("LocalQpNoSolution")

        /**
         * Cumulative count of pairwise QP solves that ended without a solution.
         */
        val PAIRWISE_QP_NO_SOLUTION_MOLECULE = SimpleMolecule("PairwiseQpNoSolution")
    }
}
