package it.unibo.collektive.control.clf

import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.control.dsl.ConstraintFormula
import it.unibo.collektive.control.dsl.ControlFunctionScope
import it.unibo.collektive.control.dsl.lessThanOrEqualTo
import it.unibo.collektive.control.dsl.expressions.dot
import it.unibo.collektive.control.dsl.expressions.minus
import it.unibo.collektive.control.dsl.expressions.squared
import it.unibo.collektive.control.dsl.expressions.squaredNorm
import it.unibo.collektive.control.dsl.expressions.times
import it.unibo.collektive.mathutils.toDoubleArray
import it.unibo.collektive.model.Target

/**
 * Discrete-time CLF constraint for goal-reaching under ZOH dynamics.
 *
 * Installed constraint (built once):
 * ```
 * 2Δt·(p_i − p_g)ᵀ·u − slack  ≤  −λ·‖p_i − p_g‖² − Δt²·u_max²
 * ```
 *
 * Both the LHS coefficients `2Δt·(p_i − p_g)[k]` and the RHS change every iteration as the robot
 * moves. The current target is retrieved through [targetProvider] on every update so the cached
 * solver model can react to target motion at runtime.
 *
 * The slack variable is mandatory for CLF feasibility and is provided by the enclosing local QP
 * as the shared node slack `ω_i`.
 *
 * @property targetProvider  supplies the current navigation goal
 * @property convergenceRate Lyapunov decrease rate λ
 * @property slackWeight     objective penalty for the slack variable (default 1.0)
 */
class GoToTargetCLF(
    override val convergenceRate: Double = 1.0,
    override val slackWeight: Double? = 1.0,
    private var targetProvider: () -> Target,
) : CLF() {

    override val name: String = "go_to_target"

    override fun syncFrom(other: ControlFunction) {
        if (other is GoToTargetCLF) {
            targetProvider = other.targetProvider
        }
    }

    override fun ControlFunctionScope.formula(): ConstraintFormula {
        val targetPosition = vector { targetProvider().position.toDoubleArray() }
        val distance = self.position - targetPosition

        return 2.0 * timeStep * dot(distance, self.u) - slack lessThanOrEqualTo
            -convergenceRate * squaredNorm(distance) - squared(timeStep) * squared(self.maxSpeed)
    }
}
