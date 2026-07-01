package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBConstr
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQConstr
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.dsl.expressions.AffineExpression
import it.unibo.collektive.model.Device
import it.unibo.collektive.solver.gurobi.GRBVector
import it.unibo.collektive.solver.gurobi.InstalledControlConstraint
import it.unibo.collektive.solver.gurobi.QpSettings

/**
 * Compiles a formula-backed control function into a reusable Gurobi constraint.
 *
 * This is the only layer of the DSL that knows about Gurobi expression objects.  It creates the
 * optional slack variable, evaluates the formula once to discover its variable structure, installs
 * the matching linear or quadratic constraint, and returns an [InstalledControlConstraint] that
 * refreshes numerical coefficients/RHS values on every solver iteration.
 */
internal fun GRBModel.installFormulaConstraint(
    name: String,
    slackName: String = name,
    selfDecision: GRBVector,
    otherDecision: GRBVector?,
    slackPolicy: SlackPolicy,
    slackWeight: Double?,
    buildFormula: ControlFunctionScope.() -> ConstraintFormula,
): InstalledControlConstraint {
    validateSlackConfiguration(slackPolicy, slackWeight, name)
    val slack = createSlack(slackPolicy, slackWeight, slackName)
    val scope = ControlFunctionScope(
        selfDecision = selfDecision,
        otherDecision = otherDecision,
        slack = slack?.let { AffineExpression.variable(it) } ?: AffineExpression.empty(),
    )
    return when (val formula = scope.buildFormula()) {
        is LinearConstraintFormula -> compileLinearFormula(name, slack, slackWeight, formula)
        is QuadraticConstraintFormula -> compileQuadraticFormula(name, slack, slackWeight, formula)
    }
}

/**
 * Fails fast when a slack configuration would silently create an unpenalized or ignored slack.
 */
private fun validateSlackConfiguration(policy: SlackPolicy, slackWeight: Double?, name: String) {
    when (policy) {
        SlackPolicy.None -> require(slackWeight == null) {
            "$name declares slackWeight=$slackWeight but its slack policy is None"
        }
        SlackPolicy.Optional -> require(slackWeight == null || slackWeight.isValidPenaltyWeight()) {
            "$name declares slackWeight=$slackWeight; slack weights must be finite and positive"
        }
        SlackPolicy.Required -> require(slackWeight != null && slackWeight.isValidPenaltyWeight()) {
            "$name requires a slack variable and must declare a finite positive slackWeight"
        }
    }
}

/**
 * Returns whether this value can be used as a meaningful quadratic penalty for a slack variable.
 */
private fun Double.isValidPenaltyWeight(): Boolean = isFinite() && this > 0.0

/**
 * Creates the slack variable requested by [SlackPolicy].
 */
private fun GRBModel.createSlack(policy: SlackPolicy, slackWeight: Double?, name: String): GRBVar? = when (policy) {
    SlackPolicy.None -> null
    SlackPolicy.Optional -> slackWeight?.let { addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_$name") }
    SlackPolicy.Required -> addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_$name")
}

/**
 * Installs the fixed variable structure of an affine formula and returns its runtime updater.
 */
private fun GRBModel.compileLinearFormula(
    name: String,
    slack: GRBVar?,
    slackWeight: Double?,
    formula: LinearConstraintFormula,
): InstalledControlConstraint {
    val variables = formula.leftHandSide.terms.map { it.variable }.distinct()
    val leftHandSideExpression = GRBLinExpr().apply {
        variables.forEach { addTerm(0.0, it) }
    }
    val constraint = addConstr(leftHandSideExpression, formula.gurobiSense, 0.0, name)
    return CompiledLinearFormula(slack, slackWeight, formula, variables, constraint)
}

/**
 * Runtime handle for a compiled affine formula.
 *
 * The installed `GRBConstr` keeps the same variables for the lifetime of the model.  On update this
 * handle recomputes the RHS and every dynamic coefficient from [FormulaRuntime].
 */
private class CompiledLinearFormula(
    override val slack: GRBVar?,
    override val slackWeight: Double?,
    private val formula: LinearConstraintFormula,
    private val variables: List<GRBVar>,
    private val constraint: GRBConstr,
) : InstalledControlConstraint {
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

/**
 * Installs the fixed quadratic structure of a formula and returns its runtime updater.
 */
private fun GRBModel.compileQuadraticFormula(
    name: String,
    slack: GRBVar?,
    slackWeight: Double?,
    formula: QuadraticConstraintFormula,
): InstalledControlConstraint {
    val leftHandSideExpression = GRBQuadExpr().apply {
        formula.leftHandSide.terms.forEach { addTerm(it.coefficient, it.first, it.second) }
    }
    val constraint = addQConstr(leftHandSideExpression, formula.gurobiSense, 0.0, name)
    return CompiledQuadraticFormula(slack, slackWeight, formula, constraint)
}

/**
 * Runtime handle for a compiled quadratic formula.
 *
 * Quadratic terms are fixed when the model is built.  The updater only refreshes the quadratic
 * constraint RHS, which is enough for the currently supported norm constraints.
 */
private class CompiledQuadraticFormula(
    override val slack: GRBVar?,
    override val slackWeight: Double?,
    private val formula: QuadraticConstraintFormula,
    private val constraint: GRBQConstr,
) : InstalledControlConstraint {
    override fun update(model: GRBModel, self: Device, otherDevice: Device?, settings: QpSettings, deltaTime: Double) {
        val runtime = FormulaRuntime(self, otherDevice, settings, deltaTime)
        constraint.set(GRB.DoubleAttr.QCRHS, formula.rightHandSide.evaluate(runtime))
    }
}
