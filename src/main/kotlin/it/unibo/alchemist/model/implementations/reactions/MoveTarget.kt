package it.unibo.alchemist.model.implementations.reactions

import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.ReactionHost
import it.unibo.alchemist.model.Time
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.alchemist.model.reactions.AbstractReaction

/**
 * One-shot environment-wide reaction that relocates the target node tagged with [targetID]
 * to ([xDest], [yDest]) at the given [occurrence] time, then removes itself from the environment.
 *
 * @param T the concentration type managed by the node
 * @param environment the simulation environment hosting this reaction
 * @param occurrence the absolute simulation time at which the target moves
 * @param xDest the destination of the target along the x-axis
 * @param yDest the destination of the target along the y-axis
 * @param targetID the ID of the target to move
 */
class MoveTarget<T>(
    val environment: Environment<T, Euclidean2DPosition>,
    occurrence: Time,
    val xDest: Double,
    val yDest: Double,
    val targetID: Int,
) : AbstractReaction<T>(occurrence) {

    override val host: ReactionHost<T> get() = environment

    override fun performModelMutation() {
        val targetNode = environment.nodes.current.find { it[SimpleMolecule("Target")].getOrNull() == targetID }
            ?: error("Target $targetID not found")
        environment.moveNodeTo(targetNode, environment.makePosition(xDest, yDest))
    }

    override fun execute() {
        super.execute()
        host.removeReaction(this)
    }

    override fun scheduleAfterInvalidation(currentTime: Time) = Unit
}
