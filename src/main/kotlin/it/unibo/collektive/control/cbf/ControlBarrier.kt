package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.solver.gurobi.GRBVector

/**
 * Pluggable barrier builder identified by [name] and exposing [add] to inject constraints.
 * Each CBF may hold a [slack] variable created during [add] and a [slackWeight] for the objective penalty.
 */
abstract class CBF: ControlFunction {
    override val name: String
        get() = "CBF"

    abstract val eta: Double // default CBF class-wide parameter (can be overridden by subclasses)

    /** Slack variable created during [add], `null` when the constraint is hard. */
    override val slack: GRBVar?
        get() = null

    /** Extension function: each CBF must implement its own logic here. */
    abstract fun GRBModel.applyCBF(uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext)

    /** Adds this CBF to [model] using local variables [uSelf], optional neighbor variables [uOther], and [context]. */
    override fun add(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext) {
        model.applyCBF(uSelf, uOther, context)
    }
}

object CBFRegistry {
    /**
     * Apply all registered CBFs for a single-agent (local) problem.
     */
    fun applyLocalCBFs(model: GRBModel, u: GRBVector, context: ControlFunctionContext, cbfs: List<CBF>) =
        cbfs.forEach { it.add(model, u, null, context) }

    /**
     * Apply all registered CBFs for a pairwise problem.
     */
    fun applyPairwiseCBFs(model: GRBModel, ui: GRBVector, uj: GRBVector, context: ControlFunctionContext, cbfs: List<CBF>) =
        cbfs.forEach { it.add(model, ui, uj, context) }
}
