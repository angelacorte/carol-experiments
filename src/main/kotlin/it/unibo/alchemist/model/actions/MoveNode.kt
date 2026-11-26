package it.unibo.alchemist.model.actions

import it.unibo.alchemist.model.Action
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.NodeReaction
import it.unibo.alchemist.model.TimeDistributedReaction
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.zeroSpeed

/**
 * Represents a movement action that updates the position of a node within a Euclidean 2D environment.
 * This class defines the motion dynamics, including velocity, boundaries, and trajectory behavior.
 *
 * @param T the concentration type managed by the node
 * @param environment the simulation environment in which the node resides
 * @param reaction the node reaction owning this action; the moved node is its host
 */
class MoveNode<T>(environment: Environment<T, Euclidean2DPosition>, reaction: NodeReaction<T>) :
    AbstractMoveNode<T, Euclidean2DPosition>(environment, reaction, false) {

    override fun getNextPosition(): Euclidean2DPosition {
        val speed = targetNode[speedMolecule].getOrNull() as? SpeedControl2D
            ?: zeroSpeed()
        val dt = targetNode[SimpleMolecule("DeltaTime")].getOrNull() as? Double
            ?: (reaction as TimeDistributedReaction<*>).meanDelay
//        node.setConcentration(speedMolecule, zeroSpeed() as T) // todo maybe it's wrong to do it here
        return Euclidean2DPosition(speed.x * dt, speed.y * dt)
    }

    override fun cloneOnNodeReaction(newReaction: NodeReaction<T>): Action<T> = MoveNode(environment, newReaction)

    /**
     * Companion object for Move Node Action.
     */
    companion object {
        /**
         * The molecule representing the velocity vector for the node.
         */
        val speedMolecule = SimpleMolecule("Velocity")
    }
}
