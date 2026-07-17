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
import it.unibo.collektive.model.Device

/**
 * Reusable Gurobi model for the pairwise QP solved on a shared edge during ADMM.
 */
class PairwiseQP private constructor(
    private val model: GRBModel,
    private val slack: GRBVar,
    private val zi: GRBVector,
    private val zj: GRBVector,
    private val pairwiseCBFs: List<CBF>,
    private val constraints: List<InstalledControlConstraint>,
) {

    /**
     * Releases the underlying Gurobi model resources.
     */
    fun dispose() = model.dispose()

    /**
     * Synchronizes the already-installed pairwise control functions with the latest runtime instances.
     *
     * The pairwise model is created once per edge and never rebuilt for the lifetime of that edge
     * (see [Solver.setupPairwiseModel]). If a pairwise CBF captures an external, time-varying
     * provider -- the same way [it.unibo.collektive.control.cbf.ObstacleAvoidanceCBF] captures an
     * `obstacleProvider` on the local model -- that provider would otherwise stay frozen at
     * edge-creation time. Call this every control period, mirroring [LocalQP.syncControlFunctions],
     * so such providers keep receiving fresh instances instead of going stale.
     *
     * Matching is done by [it.unibo.collektive.control.ControlFunction.name]; see [syncByName] for
     * the rationale and the failure modes it rules out.
     */
    fun syncControlFunctions(pairwiseCBFs: List<CBF>) {
        syncByName(this.pairwiseCBFs, pairwiseCBFs, "pairwise CBFs")
    }

    /**
     * Updates and solves the pairwise edge model for the current local and neighbor states.
     *
     * @param device current local device state.
     * @param other current neighbor device state.
     * @param incidentDuals dual variables associated with the shared edge.
     * @param settings numerical settings shared by the solver.
     * @param deltaTime control horizon expressed in seconds.
     * @return the consensus controls suggested for both endpoints.
     */
    fun updateAndSolve(
        device: Device,
        other: Device,
        incidentDuals: LocalDualUpdate,
        settings: QpSettings,
        deltaTime: Double,
    ): SuggestedControl {
        zi.applyBoundsAndWarmStart(device.maxSpeed, device.control.toDoubleArray())
        zj.applyBoundsAndWarmStart(other.maxSpeed, other.control.toDoubleArray())
        constraints.forEach { constraint -> constraint.update(model, device, other, settings, deltaTime) }
        model.setObjective(buildObjective(device, other, incidentDuals, settings), GRB.MINIMIZE)
        model.optimizeWithDiagnostics("commonModel.ilp")
        return when {
            model.get(GRB.IntAttr.SolCount) > 0 -> SuggestedControl(zi.readSpeedControl(), zj.readSpeedControl())
            else -> {
                println(
                    "Pairwise QP: no solution found (status ${model.get(GRB.IntAttr.Status)}), " +
                        "returning current controls.",
                )
                SuggestedControl(device.control, other.control)
            }
        }
    }

    private fun buildObjective(
        device: Device,
        other: Device,
        incidentDuals: LocalDualUpdate,
        settings: QpSettings,
    ): GRBQuadExpr = GRBQuadExpr().apply {
        val rho = settings.rhoADMM / 2.0
        // (ρ/2)‖z_i − (u_i + y_i)‖²  +  (ρ/2)‖z_j − (u_j + y_j)‖²
        addRhoNorm2Sq(zi, (device.control + incidentDuals.yi).toDoubleArray(), rho)
        addRhoNorm2Sq(zj, (other.control + incidentDuals.yj).toDoubleArray(), rho)
        addSlackPenalties(constraints)
        addTerm(settings.rhoSlack, slack, slack)
    }

    /**
     * Factory methods for creating a fully installed pairwise QP model.
     */
    companion object {

        /**
         * Builds a pairwise QP model with all requested edge constraints already installed.
         *
         * @param model Gurobi model to populate.
         * @param device local device used to size the first decision vector.
         * @param other neighbor device used to size the second decision vector.
         * @param pairwiseCBFs pairwise barrier functions to install on the model.
         * @return the reusable pairwise QP wrapper.
         */
        fun create(model: GRBModel, device: Device, other: Device, pairwiseCBFs: List<CBF>): PairwiseQP {
            val slack = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_pairwiseQP")
            val zi = model.addVecVar(device.position.dimension, -device.maxSpeed, device.maxSpeed, "z_ij^i")
            val zj = model.addVecVar(other.position.dimension, -other.maxSpeed, other.maxSpeed, "z_ij^j")
            val constraints = mutableListOf<InstalledControlConstraint>()
            pairwiseCBFs.forEach { cbf -> constraints += cbf.install(model, zi, zj) }
            model.update()
            return PairwiseQP(
                model = model,
                slack = slack,
                zi = zi,
                zj = zj,
                pairwiseCBFs = pairwiseCBFs,
                constraints = constraints,
            )
        }
    }
}
