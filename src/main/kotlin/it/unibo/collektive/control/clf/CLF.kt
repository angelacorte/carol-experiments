package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.dsl.ConstraintFormula
import it.unibo.collektive.control.dsl.FormulaScope
import it.unibo.collektive.control.dsl.SlackPolicy
import it.unibo.collektive.control.dsl.installFormulaConstraint
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.GRBVector

/**
 * Base class for Control Lyapunov Functions.
 *
 * Subclasses define a symbolic [formula].  The base class installs that formula into Gurobi once and
 * returns a [Constraint] that refreshes only numerical coefficients and RHS values at runtime.
 *
 * Note that CLF instances may carry dynamic goal information (e.g. a target position that moves).
 * Override [ControlFunction.syncFrom] to keep those runtime providers in sync without rebuilding the
 * model structure.
 */
abstract class CLF : ControlFunction {

    override val name: String get() = "CLF"

    /** Rate at which the Lyapunov function is forced to decrease toward zero. */
    abstract val convergenceRate: Double

    protected open val constraintName: String get() = "${name}_CLF"

    protected open val slackPolicy: SlackPolicy get() = SlackPolicy.Required

    protected abstract fun FormulaScope.formula(): ConstraintFormula

    final override fun install(model: GRBModel, selfDecision: GRBVector, otherDecision: GRBVector?): Constraint =
        model.installFormulaConstraint(
            name = constraintName,
            slackName = name,
            selfDecision = selfDecision,
            otherDecision = null,
            slackPolicy = slackPolicy,
            slackWeight = slackWeight,
        ) {
            formula()
        }
}
