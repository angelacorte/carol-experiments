package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.admm.DualParams
import it.unibo.collektive.admm.LocalDualUpdate
import it.unibo.collektive.admm.SuggestedControl
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.model.Device
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.solver.Solver

/**
 * Default Gurobi-backed [Solver] for a single device.
 *
 * Building a Gurobi model is expensive, so a model is built the first time a set of control functions is
 * seen, then reused: later `setup*` calls with the same function names only re-synchronize the installed
 * functions (dynamic providers such as targets or obstacles), and solves refresh only the numerical
 * coefficients. A different set of names (e.g. the robot switched task, or uses other pairwise constraints
 * with a given neighbor) selects or builds another model, so the device is never bound to one set.
 *
 * Values baked in at installation (e.g. `eta`, `range`, `slackWeight`) are taken from the first instance
 * installed under a name: give functions with different parameters different names.
 *
 * The [env] is not owned by the solver: it can be shared by the solvers of many devices (starting a [GRBEnv]
 * is the costly part, and may consume a license token), as long as they are never used concurrently, since
 * a [GRBEnv] is not thread-safe. Dispose the solvers with [dispose] before disposing the [env].
 * Use [defaultGurobiEnv] to create it with the license supplied by the user at runtime.
 *
 * @property settings numerical and logging configuration shared by every managed QP.
 * @property env started Gurobi environment used to build the models.
 * @property onLocalNoSolution notified when a local solve ends without a solution (previous control is kept).
 * @property onPairwiseNoSolution notified when a pairwise solve ends without a solution (current controls kept).
 */
class GurobiSolver @JvmOverloads constructor(
    override val settings: QpSettings,
    private val env: GRBEnv,
    private val onLocalNoSolution: QpNoSolutionListener = QpNoSolutionListener.IGNORE,
    private val onPairwiseNoSolution: QpNoSolutionListener = QpNoSolutionListener.IGNORE,
) : Solver {

    private val localModels = mutableMapOf<Any, LocalQP>()

    private val pairwiseModels = mutableMapOf<Any, PairwiseQP>()

    private var local: LocalQP? = null

    private var pairwise: PairwiseQP? = null

    override val isLocalModelAvailable: Boolean get() = local != null

    override val isPairwiseModelAvailable: Boolean get() = pairwise != null

    override fun setupLocalModel(device: Device, localCLFs: List<CLF>, localCBFs: List<CBF>) {
        val key = localCLFs.names() to localCBFs.names()
        local = localModels[key]?.apply { syncControlFunctions(localCLFs, localCBFs) }
            ?: LocalQP.create(newModel(), device, localCLFs, localCBFs, onLocalNoSolution)
                .also { localModels[key] = it }
    }

    override fun setupPairwiseModel(device: Device, otherDevice: Device, pairwiseCBFs: List<CBF>) {
        val key = pairwiseCBFs.names()
        pairwise = pairwiseModels[key]?.apply { syncControlFunctions(pairwiseCBFs) }
            ?: PairwiseQP.create(newModel(), device, otherDevice, pairwiseCBFs, onPairwiseNoSolution)
                .also { pairwiseModels[key] = it }
    }

    override fun <ID : Comparable<ID>> updateAndSolveLocal(
        device: Device,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        deltaTime: Double,
    ): SpeedControl2D = checkNotNull(local) { "setupLocalModel must be called before solving" }
        .updateAndSolve(device, uNominal, duals, settings, deltaTime)

    override fun updateAndSolvePairwise(
        device: Device,
        otherDevice: Device,
        duals: LocalDualUpdate,
        deltaTime: Double,
    ): SuggestedControl = checkNotNull(pairwise) { "setupPairwiseModel must be called before solving" }
        .updateAndSolve(device, otherDevice, duals, settings, deltaTime)

    /**
     * Releases every model built by this solver (not the [env]). The solver cannot be used afterwards.
     */
    fun dispose() {
        localModels.values.forEach { it.dispose() }
        pairwiseModels.values.forEach { it.dispose() }
    }

    private fun newModel(): GRBModel = GRBModel(env).also { if (settings.logEnabled) it.setupLogger() }

    private fun List<ControlFunction>.names(): Set<String> = mapTo(mutableSetOf()) { it.name }
}
