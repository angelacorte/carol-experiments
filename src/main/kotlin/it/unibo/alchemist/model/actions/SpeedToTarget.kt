package it.unibo.alchemist.model.actions

import it.unibo.alchemist.model.Action
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.Reaction
import it.unibo.alchemist.model.movestrategies.routing.StraightLine
import it.unibo.alchemist.model.movestrategies.target.SpeedFromMolecule
import it.unibo.alchemist.model.movestrategies.target.TakeTargetFromMolecule

/**
 * An action that moves a node towards a dynamically computed target at a speed determined by node properties.
 *
 * This action uses a straight-line routing strategy, a target computed from node molecules, and a speed
 * determined by the node's velocity and reaction. The movement is interpolated to ensure the node does not
 * exceed its allowed movement per step.
 *
 * @param T the type of the concentrations in the environment
 * @param P the type of position used in the environment
 * @property environment the environment in which the node moves
 * @property node the node to move
 * @property reaction the reaction associated with the node
 */
class SpeedToTarget<T, P : Position<P>>(environment: Environment<T, P>, node: Node<T>, reaction: Reaction<T>) :
    AbstractConfigurableMoveNode<T, P>(
        environment,
        node,
        StraightLine<T, P>(),
        TakeTargetFromMolecule(environment, node),
        SpeedFromMolecule(node, reaction),
        true,
    ) {
    override fun cloneAction(node: Node<T>, reaction: Reaction<T>): Action<T> =
        SpeedToTarget(environment, node, reaction)

    override fun interpolatePositions(current: P?, target: P?, maxWalk: Double): P? = current?.let { curr ->
        target?.let { goal ->
            val distance = curr.distanceTo(goal)
            when {
                distance == 0.0 || maxWalk == 0.0 -> goal
                else -> {
                    environment.makePosition(
                        curr.coordinates.mapIndexed { index, value ->
                            value + (goal.coordinates[index] - value) * maxWalk / distance
                        },
                    ) as P
                }
            }
        }
    } ?: current
}
