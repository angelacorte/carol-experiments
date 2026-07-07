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
import it.unibo.collektive.model.Device
import it.unibo.collektive.model.SpeedControl2D

/**
 * Reusable Gurobi model for the single-device QP solved during each ADMM iteration.
 */
class LocalQP private constructor(
    private val model: GRBModel,
    private val u: GRBVector,
    private val slack: GRBVar,
    private val localCLFs: List<CLF>,
    private val localCBFs: List<CBF>,
    private val constraints: List<InstalledControlConstraint>,
) {

    /**
     * Releases the underlying Gurobi model resources.
     */
    fun dispose() = model.dispose()

    /**
     * Synchronizes the already-installed control functions with the latest runtime instances.
     *
     * Matching is done by [it.unibo.collektive.control.ControlFunction.name] rather than by list
     * position -- see [syncByName] for the rationale and the failure modes it rules out.
     */
    fun syncControlFunctions(localCLFs: List<CLF>, localCBFs: List<CBF>) {
        syncByName(this.localCLFs, localCLFs, "local CLFs")
        syncByName(this.localCBFs, localCBFs, "local CBFs")
    }

    /**
     * Updates variable bounds, refreshes constraint coefficients, solves the QP, and returns the new control.
     *
     * @param device current local device state.
     * @param uNominal nominal control used by the quadratic objective.
     * @param duals per-neighbor ADMM state used to build the consensus penalty.
     * @param settings numerical settings shared by the solver.
     * @param deltaTime control horizon expressed in seconds.
     * @return the control selected by the optimizer, or the previous control if no solution is found.
     */
    fun <ID : Comparable<ID>> updateAndSolve(
        device: Device,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        settings: QpSettings,
        deltaTime: Double,
    ): SpeedControl2D {
        u.applyBoundsAndWarmStart(device.maxSpeed, device.control.toDoubleArray())
        constraints.forEach { constraint ->
            constraint.update(model, device, settings = settings, deltaTime = deltaTime)
        }
        model.setObjective(buildObjective(uNominal, duals, settings), GRB.MINIMIZE)
        model.optimizeWithDiagnostics("localModel.ilp")
        return when {
            model.get(GRB.IntAttr.SolCount) > 0 -> u.readSpeedControl()
            else -> {
                println(
                    "Local QP: no solution found (status ${model.get(GRB.IntAttr.Status)}), " +
                        "returning previous control.",
                )
                device.control
            }
        }
    }

    private fun buildObjective( // todo this should be taken from outside, can be different by different simulations
        uNominal: DoubleArray,
        duals: Map<*, DualParams>,
        settings: QpSettings,
    ): GRBQuadExpr = GRBQuadExpr().apply {
        addRhoNorm2Sq(u, uNominal)
        addSlackPenalties(constraints)
        // NOTE: `slack` is not referenced by any installed constraint, so this term is currently
        // inert (the optimizer always drives it to 0). Left in place on purpose.
        addTerm(settings.rhoSlack, slack, slack)
        duals.forEach { (_, value) ->
            val suggested = value.suggestedControl.zi.toDoubleArray()
            val residual = value.localDualUpdate.yi.toDoubleArray()
            addRhoNorm2Sq(u, suggested - residual, settings.rhoADMM / 2.0)
        }
    }

    /**
     * Factory methods for creating a fully installed local QP model.
     */
    companion object {
        /**
         * Builds a local QP model with all requested control functions already installed.
         *
         * @param model Gurobi model to populate.
         * @param device device used to size the control vector and bounds.
         * @param localCLFs local CLF constraints to install.
         * @param localCBFs local CBF constraints to install.
         * @return the reusable local QP wrapper.
         */
        fun create(model: GRBModel, device: Device, localCLFs: List<CLF>, localCBFs: List<CBF>): LocalQP {
            val u = model.addVecVar(device.position.dimension, -device.maxSpeed, device.maxSpeed, "u")
            val slack = model.addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_localQP")
            val installed = mutableListOf<InstalledControlConstraint>()
            localCLFs.forEach { clf -> installed += clf.install(model, u, null) }
            localCBFs.forEach { cbf -> installed += cbf.install(model, u, null) }
            model.update()
            return LocalQP(
                model = model,
                u = u,
                slack = slack,
                localCLFs = localCLFs,
                localCBFs = localCBFs,
                constraints = installed,
            )
        }
    }
}
