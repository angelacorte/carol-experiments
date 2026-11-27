package it.unibo.collektive.alchemist.device.sensors.impl

import it.unibo.alchemist.model.Capability
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.collektive.alchemist.device.sensors.LocationSensor
import it.unibo.collektive.model.Coordinate
import it.unibo.collektive.model.Target

/**
 * An implementation of a location sensor property for nodes in an Alchemist environment.
 *
 * @param T the concentration type managed by the node
 * @param P the position type used in the environment
 * @property environment the simulation environment
 * @property node the node associated with this sensor property
 * @property random an instance of a random number generator for simulating measurement noise
 * @property stdDev the standard deviation for Gaussian noise added to target positions
 */
class LocationSensorProperty<T, P : Position<P>>(
    private val environment: Environment<T, P>,
    override val node: Node<T>,
) : LocationSensor,
    Capability<T> {

    override fun cloneOnNewNode(node: Node<T>): Capability<T> = LocationSensorProperty(environment, node)

    override fun coordinates(): Coordinate {
        val position = environment.positionOf(node).current
        return Coordinate(position[0], position[1])
    }

    override fun surroundings(): List<Coordinate> = environment.neighborhoodOf(node).current.neighbors.map { node ->
        environment.positionOf(node).current.let { Coordinate(it[0], it[1]) }
    }

    override fun targetsPosition(): List<Target> = environment.nodes.current.filter { node ->
        node.contains(SimpleMolecule("Target"))
    }.map { target ->
        (target[SimpleMolecule("Target")].getOrNull() as Number) to environment.positionOf(target)
    }.map { (id, position) ->
        val currentPosition = position.current
        Target(currentPosition[0], currentPosition[1], id)
    }
}
