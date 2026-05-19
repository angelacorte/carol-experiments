package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.dsl.ConstraintFormula
import it.unibo.collektive.control.dsl.ControlFunctionScope
import it.unibo.collektive.control.dsl.SlackPolicy
import it.unibo.collektive.control.dsl.installFormulaConstraint
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.InstalledControlConstraint

/**
 * Base class for Control Barrier Functions.
 *
 * Subclasses define a symbolic [formula].  The base class installs that formula into Gurobi once and
 * returns an [InstalledControlConstraint] that refreshes only numerical coefficients and RHS values at runtime.
 */
abstract class CBF : ControlFunction {

    override val name: String get() = "CBF"

    /** Decay-rate parameter governing how strictly the barrier is enforced. */
    abstract val eta: Double

    /**
     * Name of the Gurobi constraint generated for this CBF.
     *
     * By default it matches [name], but subclasses can override it when they need to preserve a
     * legacy solver/debug name independently of the control function name.
     */
    protected open val constraintName: String get() = name

    /**
     * Slack creation strategy used when compiling this CBF formula.
     */
    protected open val slackPolicy: SlackPolicy
        get() = if (slackWeight == null) SlackPolicy.None else SlackPolicy.Optional

    /**
     * Mathematical formula enforced by this CBF.
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
        otherDecision = otherDecision,
        slackPolicy = slackPolicy,
        slackWeight = slackWeight,
    ) {
        formula()
    }
}
