package it.unibo.collektive.control.clf

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.dsl.ConstraintFormula
import it.unibo.collektive.control.dsl.ControlFunctionScope
import it.unibo.collektive.control.dsl.SlackPolicy
import it.unibo.collektive.control.dsl.installFormulaConstraint
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.InstalledControlConstraint

/**
 * Base class for Control Lyapunov Functions.
 *
 * Subclasses define a symbolic [formula].  The base class installs that formula into Gurobi once and
 * returns an [InstalledControlConstraint] that refreshes only numerical coefficients and RHS values at runtime.
 *
 * Note that CLF instances may carry dynamic goal information (e.g. a target position that moves).
 * Override [ControlFunction.syncFrom] to keep those runtime providers in sync without rebuilding the
 * model structure.
 */
abstract class CLF : ControlFunction {

    override val name: String get() = "CLF"

    /** Rate at which the Lyapunov function is forced to decrease toward zero. */
    abstract val convergenceRate: Double

    /**
     * Name of the Gurobi constraint generated for this CLF.
     */
    protected open val constraintName: String get() = "${name}_CLF"

    /**
     * Slack creation strategy used when compiling this CLF formula.
     *
     * CLFs default to a required slack because goal-reaching constraints may otherwise become
     * infeasible under hard safety constraints.
     */
    protected open val slackPolicy: SlackPolicy get() = SlackPolicy.Required

    /**
     * Mathematical formula enforced by this CLF.
     *
     * The formula is built once during model installation.  Any state-dependent value must be
     * represented through [ControlFunctionScope] runtime expressions rather than read eagerly.
     */
    protected abstract fun ControlFunctionScope.formula(): ConstraintFormula

    final override fun install(
        model: GRBModel,
        selfDecision: GRBVector,
        otherDecision: GRBVector?,
    ): InstalledControlConstraint = model.installFormulaConstraint(
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
