package it.unibo.collektive.control.dsl

import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Device
import it.unibo.collektive.solver.gurobi.GRBVector

enum class SlackPolicy {
    None,
    Optional,
    Required,
}

class AgentExpression internal constructor(
    decision: GRBVector,
    private val device: (FormulaRuntime) -> Device,
) {
    val u: DecisionVector = DecisionVector(decision)

    val position: VectorExpression = VectorExpression { runtime -> device(runtime).position.toDoubleArray() }

    val safeMargin: ScalarExpression = ScalarExpression { runtime -> device(runtime).safeMargin }

    val maxSpeed: ScalarExpression = ScalarExpression { runtime -> device(runtime).maxSpeed }
}

sealed class ConstraintFormulaScope protected constructor(
    selfDecision: GRBVector,
    slackVariable: GRBVar?,
) {
    val self: AgentExpression = AgentExpression(selfDecision) { it.self }

    val timeStep: ScalarExpression = ScalarExpression { it.deltaTime }

    val slack: AffineExpression = slackVariable?.let {
        AffineExpression(listOf(LinearTerm(it, scalar(1.0))))
    } ?: AffineExpression()

    fun scalar(block: FormulaRuntime.() -> Double): ScalarExpression =
        ScalarExpression { runtime -> runtime.block() }

    fun vector(block: FormulaRuntime.() -> DoubleArray): VectorExpression =
        VectorExpression { runtime -> runtime.block() }
}

class LocalFormulaScope internal constructor(
    selfDecision: GRBVector,
    slackVariable: GRBVar?,
) : ConstraintFormulaScope(selfDecision, slackVariable)

class PairwiseFormulaScope internal constructor(
    selfDecision: GRBVector,
    otherDecision: GRBVector,
    slackVariable: GRBVar?,
) : ConstraintFormulaScope(selfDecision, slackVariable) {
    val other: AgentExpression = AgentExpression(otherDecision) {
        checkNotNull(it.other) { "Pairwise formula requires the neighbor device at update time" }
    }
}

class FormulaScope internal constructor(
    private val selfDecision: GRBVector,
    private val otherDecision: GRBVector?,
    private val slackVariable: GRBVar?,
) {
    fun local(block: LocalFormulaScope.() -> ConstraintFormula): ConstraintFormula =
        LocalFormulaScope(selfDecision, slackVariable).block()

    fun pairwise(block: PairwiseFormulaScope.() -> ConstraintFormula): ConstraintFormula {
        val decision = checkNotNull(otherDecision) { "Pairwise formula requires the neighbor decision vector" }
        return PairwiseFormulaScope(selfDecision, decision, slackVariable).block()
    }
}
