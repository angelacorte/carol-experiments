package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBConstr
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQConstr
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.model.Device
import it.unibo.collektive.solver.gurobi.Constraint
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.QpSettings

internal fun GRBModel.installFormulaConstraint(
    name: String,
    slackName: String = name,
    selfDecision: GRBVector,
    otherDecision: GRBVector?,
    slackPolicy: SlackPolicy,
    slackWeight: Double?,
    buildFormula: FormulaScope.() -> ConstraintFormula,
): Constraint {
    val slack = createSlack(slackPolicy, slackWeight, slackName)
    return when (val formula = FormulaScope(selfDecision, otherDecision, slack).buildFormula()) {
        is LinearConstraintFormula -> installLinearFormula(name, slack, slackWeight, formula)
        is QuadraticConstraintFormula -> installQuadraticFormula(name, slack, slackWeight, formula)
    }
}

private fun GRBModel.createSlack(policy: SlackPolicy, slackWeight: Double?, name: String): GRBVar? =
    when (policy) {
        SlackPolicy.None -> null
        SlackPolicy.Optional -> slackWeight?.let { addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_$name") }
        SlackPolicy.Required -> addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_$name")
    }

private fun GRBModel.installLinearFormula(
    name: String,
    slack: GRBVar?,
    slackWeight: Double?,
    formula: LinearConstraintFormula,
): Constraint {
    val variables = formula.leftHandSide.terms.map { it.variable }.distinct()
    val leftHandSideExpression = GRBLinExpr().apply {
        variables.forEach { addTerm(0.0, it) }
    }
    val constraint = addConstr(leftHandSideExpression, formula.sense.gurobiSense, 0.0, name)
    return LinearFormulaConstraint(slack, slackWeight, formula, variables, constraint)
}

private class LinearFormulaConstraint(
    override val slack: GRBVar?,
    override val slackWeight: Double?,
    private val formula: LinearConstraintFormula,
    private val variables: List<GRBVar>,
    private val constraint: GRBConstr,
) : Constraint {
    override fun update(model: GRBModel, self: Device, otherDevice: Device?, settings: QpSettings, deltaTime: Double) {
        val runtime = FormulaRuntime(self, otherDevice, settings, deltaTime)
        constraint.set(
            GRB.DoubleAttr.RHS,
            formula.rightHandSide.evaluate(runtime) - formula.leftHandSide.constant.evaluate(runtime),
        )
        val coefficients = variables.associateWithTo(LinkedHashMap()) { 0.0 }
        formula.leftHandSide.terms.forEach { term ->
            coefficients[term.variable] = coefficients.getValue(term.variable) + term.coefficient.evaluate(runtime)
        }
        coefficients.forEach { (variable, coefficient) ->
            model.chgCoeff(constraint, variable, coefficient)
        }
    }
}

private fun GRBModel.installQuadraticFormula(
    name: String,
    slack: GRBVar?,
    slackWeight: Double?,
    formula: QuadraticConstraintFormula,
): Constraint {
    val leftHandSideExpression = GRBQuadExpr().apply {
        formula.leftHandSide.terms.forEach { addTerm(it.coefficient, it.first, it.second) }
    }
    val constraint = addQConstr(leftHandSideExpression, formula.sense.gurobiSense, 0.0, name)
    return QuadraticFormulaConstraint(slack, slackWeight, formula, constraint)
}

private class QuadraticFormulaConstraint(
    override val slack: GRBVar?,
    override val slackWeight: Double?,
    private val formula: QuadraticConstraintFormula,
    private val constraint: GRBQConstr,
) : Constraint {
    override fun update(model: GRBModel, self: Device, otherDevice: Device?, settings: QpSettings, deltaTime: Double) {
        val runtime = FormulaRuntime(self, otherDevice, settings, deltaTime)
        constraint.set(GRB.DoubleAttr.QCRHS, formula.rightHandSide.evaluate(runtime))
    }
}
