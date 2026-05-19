package it.unibo.collektive.control.cbf

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.dsl.ConstraintFormula
import it.unibo.collektive.control.dsl.FormulaScope
import it.unibo.collektive.control.dsl.SlackPolicy
import it.unibo.collektive.control.dsl.installFormulaConstraint
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.GRBVector

/**
 * Base class for Control Barrier Functions.
 *
 * Subclasses define a symbolic [formula].  The base class installs that formula into Gurobi once and
 * returns a [Constraint] that refreshes only numerical coefficients and RHS values at runtime.
 */
abstract class CBF : ControlFunction {

    override val name: String get() = "CBF"

    /** Decay-rate parameter governing how strictly the barrier is enforced. */
    abstract val eta: Double

    protected open val constraintName: String get() = name

    protected open val slackPolicy: SlackPolicy
        get() = if (slackWeight == null) SlackPolicy.None else SlackPolicy.Optional

    protected abstract fun FormulaScope.formula(): ConstraintFormula

    final override fun install(model: GRBModel, selfDecision: GRBVector, otherDecision: GRBVector?): Constraint =
        model.installFormulaConstraint(
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
