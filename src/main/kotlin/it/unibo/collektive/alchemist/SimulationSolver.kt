package it.unibo.collektive.alchemist

import com.github.benmanes.caffeine.cache.Caffeine
import com.github.benmanes.caffeine.cache.LoadingCache
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Layer
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.molecules.SimpleMolecule
import it.unibo.collektive.alchemist.device.sensors.impl.SettingsProperty
import it.unibo.collektive.solver.Solver
import it.unibo.collektive.solver.gurobi.QpSettings

/**
 * Stores one [Solver] instance per simulation environment so QP models can be reused across steps.
 */
object SimulationSolver {
    private val activeSolver: LoadingCache<Environment<*, *>, Solver> = Caffeine.newBuilder()
        .weakKeys()
        .build {
            val layer: Layer<*, *> = requireNotNull(it.layers[SimpleMolecule("QPSettings")])
            val origin: Position<*> = it.makePosition(0, 0)
            layer as Layer<*, Position<*>>
            val settings = layer.getValue(origin)
            require(settings is QpSettings)
            Solver(settings)
        }

    /**
     * Returns the solver already associated with this environment.
     *
     * @throws IllegalStateException if no solver has been created for the environment yet.
     */
    val Environment<*, *>.solver: Solver
        get() = activeSolver[this]
}

/**
 * Stores one [Solver] instance per node of the simulation so QP models can be reused across steps.
 */
object NodeSolver {
    private val activeSolver: LoadingCache<Node<*>, Solver> = Caffeine.newBuilder()
        .weakKeys()
        .build { node ->
            val prop: SettingsProperty<*, *> = node.properties.first {
                it is SettingsProperty<*, *>
            } as SettingsProperty<*, *>
            Solver(prop.settings)
        }

    /**
     * Returns the solver already associated with this environment.
     *
     * @throws IllegalStateException if no solver has been created for the environment yet.
     */
    val Node<*>.solver: Solver
        get() = activeSolver[this]
}
