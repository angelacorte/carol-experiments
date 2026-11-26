package it.unibo.alchemist.model.actions

import it.unibo.alchemist.model.Action
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.NodeReaction
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.movestrategies.routing.StraightLine
import it.unibo.alchemist.model.movestrategies.target.SpeedFromMolecule
import it.unibo.alchemist.model.movestrategies.target.TakeTargetFromMolecule
import it.unibo.alchemist.model.positions.Euclidean2DPosition

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
 * @param reaction the node reaction owning this action; the moved node is its host
 */
class SpeedToTarget<T, P : Position<P>>(environment: Environment<T, Euclidean2DPosition>, reaction: NodeReaction<T>) :
    EuclideanConfigurableMoveNode<T, Euclidean2DPosition>(
        environment = environment,
        reaction = reaction,
        routingStrategy = StraightLine(),
        targetSelectionStrategy = TakeTargetFromMolecule(environment, reaction.host),
        speedSelectionStrategy = SpeedFromMolecule(reaction.host, environment),
    ) {
    override fun cloneOnNodeReaction(newReaction: NodeReaction<T>): Action<T> =
        SpeedToTarget<T, P>(environment, newReaction)
}
