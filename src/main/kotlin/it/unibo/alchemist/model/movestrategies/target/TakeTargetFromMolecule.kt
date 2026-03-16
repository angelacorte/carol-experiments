package it.unibo.alchemist.model.movestrategies.target

import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.alchemist.model.movestrategies.TargetSelectionStrategy
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.zeroSpeed

data class TakeTargetFromMolecule<T, P : Position<P>>(private val environment: Environment<T,P>, private val node: Node<T>) : TargetSelectionStrategy<T, P> {
    override fun getTarget(): P? {
        return if(node.contains(SimpleMolecule("Robot"))) {
            val speed = node.getConcentration(SimpleMolecule("Velocity")) as? SpeedControl2D ?: zeroSpeed()
            environment.makePosition(speed.x * 10, speed.y * 10)
//            val targetID = node.getConcentration(SimpleMolecule("TargetID")) as? Int
//                ?: error("Node ${node.id} contains a Robot molecule but no TargetID molecule.")
//            environment.nodes.filter { node ->
//                node.contains(SimpleMolecule("Target")) &&  node.getConcentration(SimpleMolecule("Target")) as Int? == targetID
//            }.map { target ->
//                environment.getPosition(target)
//            }.firstOrNull()
        } else {
            null
        }
    }
}
