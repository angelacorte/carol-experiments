package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.GRBVector

abstract class FormulaCBF : CBF() {

    protected open val constraintName: String get() = name

    protected open val slackPolicy: SlackPolicy
        get() = if (slackWeight == null) SlackPolicy.None else SlackPolicy.Optional

    protected abstract fun FormulaScope.formula(): ConstraintFormula

    final override fun GRBModel.installCBF(selfDecision: GRBVector, otherDecision: GRBVector?): Constraint =
        installFormulaConstraint(
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

abstract class FormulaCLF : CLF() {

    protected open val constraintName: String get() = "${name}_CLF"

    protected open val slackPolicy: SlackPolicy get() = SlackPolicy.Required

    protected abstract fun FormulaScope.formula(): ConstraintFormula

    final override fun GRBModel.installCLF(selfDecision: GRBVector): Constraint =
        installFormulaConstraint(
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
