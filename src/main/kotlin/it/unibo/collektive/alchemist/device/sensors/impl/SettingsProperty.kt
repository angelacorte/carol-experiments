package it.unibo.collektive.alchemist.device.sensors.impl

import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.NodeProperty
import it.unibo.alchemist.model.Position
import it.unibo.collektive.alchemist.device.SimulationQpSettings

/**
 * An [NodeProperty] exposing the simulation QP settings to the node.
 *
 * @property settings the shared [SimulationQpSettings] instance for this node.
 */
class SettingsProperty<T : Any, P : Position<P>>(
    private val environment: Environment<T, P>,
    override val node: Node<T>,
    val settings: SimulationQpSettings,
) : NodeProperty<T> {

    override fun cloneOnNewNode(node: Node<T>): NodeProperty<T> = SettingsProperty(environment, node, settings)
}
