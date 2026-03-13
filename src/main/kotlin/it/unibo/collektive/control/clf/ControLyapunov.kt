package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunctionContext
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.solver.gurobi.GRBVector

abstract class CLF: ControlFunction {
    override val name: String = "CLF"

    override val slack: GRBVar?
        get() = null

    abstract val convergenceRate: Double

    override fun add(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, context: ControlFunctionContext) {
        model.applyCLF(uSelf, context)
    }

    abstract fun GRBModel.applyCLF(uSelf: GRBVector, context: ControlFunctionContext)
}

object CLFRegistry {
    /**
     * Apply all registered CLFs for a single-agent (local) problem.
     */
    fun applyLocalCLFs(model: GRBModel, u: GRBVector, context: ControlFunctionContext, clfs: List<CLF>) =
        clfs.forEach { it.add(model, u, null, context) }
}
