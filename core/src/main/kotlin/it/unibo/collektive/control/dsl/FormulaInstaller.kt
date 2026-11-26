package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBConstr
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQConstr
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.dsl.expressions.AffineExpression
import it.unibo.collektive.control.dsl.expressions.LinearTerm
import it.unibo.collektive.control.dsl.expressions.RuntimeScalar
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
 *
 * Slack handling is derived directly from [slackWeight]: a slack variable exists if and only if a
 * weight is declared.  [requiresSlack] additionally forces a weight to be present (CLFs), and
 * quadratic formulas reject slack because `GRBQConstr` cannot host it on the left-hand side.
 */
internal fun GRBModel.installFormulaConstraint(
    name: String,
    slackName: String = name,
    selfDecision: GRBVector,
    otherDecision: GRBVector?,
    requiresSlack: Boolean,
    slackWeight: Double?,
    buildFormula: ControlFunctionScope.() -> ConstraintFormula,
): InstalledControlConstraint {
    require(slackWeight == null || (slackWeight.isFinite() && slackWeight > 0.0)) {
        "$name declares slackWeight=$slackWeight; slack weights must be finite and positive"
    }
    require(!requiresSlack || slackWeight != null) {
        "$name requires a slack variable and must declare a finite positive slackWeight"
    }
    val slack = slackWeight?.let { addVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "slack_$slackName") }
    val scope = ControlFunctionScope(
        selfDecision = selfDecision,
        otherDecision = otherDecision,
        slack = slack?.let { AffineExpression.variable(it) } ?: AffineExpression.empty(),
    )
    return when (val formula = scope.buildFormula()) {
        is LinearConstraintFormula -> compileLinearFormula(name, slack, slackWeight, formula)
        is QuadraticConstraintFormula -> {
            require(slack == null) { "$name: slack is not supported on quadratic constraints" }
            compileQuadraticFormula(name, formula)
        }
    }
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
    val coefficientsByVariable = formula.leftHandSide.terms.groupBy(LinearTerm::variable, LinearTerm::coefficient)
    val leftHandSideExpression = GRBLinExpr().apply {
        coefficientsByVariable.keys.forEach { addTerm(0.0, it) }
    }
    val constraint = addConstr(leftHandSideExpression, formula.gurobiSense, 0.0, name)
    return CompiledLinearFormula(slack, slackWeight, formula, coefficientsByVariable, constraint)
}

/**
 * Runtime handle for a compiled affine formula.
 *
 * The installed `GRBConstr` keeps the same variables for the lifetime of the model.  The grouping
 * of runtime coefficients by variable is computed once at installation; on update each group is
 * re-evaluated against the current [FormulaRuntime] and pushed in-place with [GRBModel.chgCoeff].
 */
private class CompiledLinearFormula(
    override val slack: GRBVar?,
    override val slackWeight: Double?,
    private val formula: LinearConstraintFormula,
    private val coefficientsByVariable: Map<GRBVar, List<RuntimeScalar>>,
    private val constraint: GRBConstr,
) : InstalledControlConstraint {
    override fun update(model: GRBModel, self: Device, otherDevice: Device?, settings: QpSettings, deltaTime: Double) {
        val runtime = FormulaRuntime(self, otherDevice, settings, deltaTime)
        constraint.set(
            GRB.DoubleAttr.RHS,
            formula.rightHandSide.evaluate(runtime) - formula.leftHandSide.constant.evaluate(runtime),
        )
        coefficientsByVariable.forEach { (variable, coefficients) ->
            model.chgCoeff(constraint, variable, coefficients.sumOf { it.evaluate(runtime) })
        }
    }
}

/**
 * Installs the fixed quadratic structure of a formula and returns its runtime updater.
 */
private fun GRBModel.compileQuadraticFormula(
    name: String,
    formula: QuadraticConstraintFormula,
): InstalledControlConstraint {
    val leftHandSideExpression = GRBQuadExpr().apply {
        formula.leftHandSide.terms.forEach { addTerm(it.coefficient, it.first, it.second) }
    }
    val constraint = addQConstr(leftHandSideExpression, formula.gurobiSense, 0.0, name)
    return CompiledQuadraticFormula(formula, constraint)
}

/**
 * Runtime handle for a compiled quadratic formula.
 *
 * Quadratic terms are fixed when the model is built.  The updater only refreshes the quadratic
 * constraint RHS, which is enough for the currently supported norm constraints.
 */
private class CompiledQuadraticFormula(
    private val formula: QuadraticConstraintFormula,
    private val constraint: GRBQConstr,
) : InstalledControlConstraint {
    override val slack: GRBVar? = null

    override val slackWeight: Double? = null

    override fun update(model: GRBModel, self: Device, otherDevice: Device?, settings: QpSettings, deltaTime: Double) {
        val runtime = FormulaRuntime(self, otherDevice, settings, deltaTime)
        constraint.set(GRB.DoubleAttr.QCRHS, formula.rightHandSide.evaluate(runtime))
    }
}
