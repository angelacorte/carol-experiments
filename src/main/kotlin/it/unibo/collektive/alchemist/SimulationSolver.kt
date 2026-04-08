package it.unibo.collektive.alchemist

import com.github.benmanes.caffeine.cache.Caffeine
import com.github.benmanes.caffeine.cache.LoadingCache
import it.unibo.alchemist.model.Environment
import it.unibo.collektive.alchemist.device.SimulationQpSettings
import it.unibo.collektive.solver.Solver
import it.unibo.collektive.solver.gurobi.QpSettings

/**
 * Stores one [Solver] instance per simulation environment so QP models can be reused across steps.
 */
object SimulationSolver {
    private val activeSolver: LoadingCache<Environment<*, *>, Solver> = Caffeine.newBuilder()
        .weakKeys()
        .build { key -> Solver(SimulationQpSettings()) } // todo doubt

    /**
     * Returns the solver already associated with this environment.
     *
     * @throws IllegalStateException if no solver has been created for the environment yet.
     */
    val Environment<*, *>.solver: Solver
        get() = activeSolver.getIfPresent(this) ?: error("Could not find solver for $this")

    /**
     * Returns the solver associated with this environment, creating and caching one when needed.
     *
     * @param settings configuration used when a new solver must be created.
     */
    fun Environment<*, *>.solver(settings: QpSettings): Solver =
        activeSolver.getIfPresent(this) ?: Solver(settings).also { activeSolver.put(this, it) }
}
