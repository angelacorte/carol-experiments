package it.unibo.collektive.alchemist.device.sensors.impl

import com.gurobi.gurobi.GRBEnv
import it.unibo.alchemist.model.Capability
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.collektive.admm.DualParams
import it.unibo.collektive.admm.LocalDualUpdate
import it.unibo.collektive.admm.SuggestedControl
import it.unibo.collektive.alchemist.SimulationSolver
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.model.Device
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.solver.Solver
import it.unibo.collektive.solver.gurobi.GurobiSolver
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.solver.gurobi.defaultGurobiEnv

/**
 * Alchemist [Capability] giving the node its own [GurobiSolver], so every node can install its own
 * control functions (see [GurobiSolver] for how models are reused).
 *
 * Starting a [GRBEnv] is the expensive part, so all the nodes of the same [environment] share one through
 * [SimulationSolver], created by the [envFactory] of the first node that solves.
 *
 * Solves of this node that end without a usable solution are counted in the
 * [LOCAL_QP_NO_SOLUTION_MOLECULE] and [PAIRWISE_QP_NO_SOLUTION_MOLECULE] molecules, so exports can tell
 * clean runs from degraded ones.
 *
 * @property settings numerical and logging configuration of this node's QPs.
 * @property envFactory creates the started [GRBEnv] shared by the environment.
 */
class SolverProperty<T, P : Position<P>> @JvmOverloads constructor(
    private val environment: Environment<T, P>,
    override val node: Node<T>,
    override val settings: QpSettings,
    private val envFactory: (QpSettings) -> GRBEnv = { defaultGurobiEnv(it.logEnabled) },
) : Solver,
    Capability<T> {

    private var localNoSolutionCount: Long = 0

    private var pairwiseNoSolutionCount: Long = 0

    private val solver: GurobiSolver by lazy {
        SimulationSolver.newSolver(
            environment,
            settings,
            createEnv = { envFactory(settings) },
            onLocalNoSolution = { recordCount(LOCAL_QP_NO_SOLUTION_MOLECULE, ++localNoSolutionCount) },
            onPairwiseNoSolution = { recordCount(PAIRWISE_QP_NO_SOLUTION_MOLECULE, ++pairwiseNoSolutionCount) },
        )
    }

    init {
        recordCount(LOCAL_QP_NO_SOLUTION_MOLECULE, 0)
        recordCount(PAIRWISE_QP_NO_SOLUTION_MOLECULE, 0)
    }

    override val isLocalModelAvailable: Boolean get() = solver.isLocalModelAvailable

    override val isPairwiseModelAvailable: Boolean get() = solver.isPairwiseModelAvailable

    override fun setupLocalModel(device: Device, localCLFs: List<CLF>, localCBFs: List<CBF>) =
        solver.setupLocalModel(device, localCLFs, localCBFs)

    override fun setupPairwiseModel(device: Device, otherDevice: Device, pairwiseCBFs: List<CBF>) =
        solver.setupPairwiseModel(device, otherDevice, pairwiseCBFs)

    override fun <ID : Comparable<ID>> updateAndSolveLocal(
        device: Device,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        deltaTime: Double,
    ): SpeedControl2D = solver.updateAndSolveLocal(device, uNominal, duals, deltaTime)

    override fun updateAndSolvePairwise(
        device: Device,
        otherDevice: Device,
        duals: LocalDualUpdate,
        deltaTime: Double,
    ): SuggestedControl = solver.updateAndSolvePairwise(device, otherDevice, duals, deltaTime)

    override fun cloneOnNewNode(node: Node<T>): Capability<T> = SolverProperty(environment, node, settings, envFactory)

    private fun recordCount(molecule: SimpleMolecule, count: Long) {
        @Suppress("UNCHECKED_CAST")
        node[molecule] = count as T
    }

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
