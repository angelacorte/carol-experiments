package it.unibo.collektive.alchemist

import com.github.benmanes.caffeine.cache.Cache
import com.github.benmanes.caffeine.cache.Caffeine
import com.gurobi.gurobi.GRBEnv
import it.unibo.alchemist.boundary.OutputMonitor
import it.unibo.alchemist.model.Environment
import it.unibo.alchemist.model.Position
import it.unibo.alchemist.model.Time
import it.unibo.collektive.solver.gurobi.GurobiSolver
import it.unibo.collektive.solver.gurobi.QpNoSolutionListener
import it.unibo.collektive.solver.gurobi.QpSettings

/**
 * Caches one started [GRBEnv] per simulation environment, so that the solvers of all its nodes share it.
 *
 * Starting a [GRBEnv] is the expensive part (and may consume a license token), while each node keeps its own
 * [GurobiSolver] and models, so nodes can install different control functions. Parallel simulations of a batch
 * get one [GRBEnv] each, as it is not thread-safe. Solvers and environment are disposed when the simulation
 * finishes.
 */
object SimulationSolver {

    private class SharedEnv(val env: GRBEnv) {
        val solvers = mutableListOf<GurobiSolver>()

        fun dispose() {
            solvers.forEach { it.dispose() }
            env.dispose()
        }
    }

    private val sharedEnvs: Cache<Environment<*, *>, SharedEnv> = Caffeine.newBuilder().weakKeys().build()

    /**
     * Creates a [GurobiSolver] for a node of [simulationEnvironment], built on the [GRBEnv] shared by the whole
     * environment. The [GRBEnv] is created by [createEnv] the first time, and must be requested once the
     * simulation exists (i.e., not while the scenario is being loaded).
     */
    fun <T, P : Position<P>> newSolver(
        simulationEnvironment: Environment<T, P>,
        settings: QpSettings,
        createEnv: () -> GRBEnv,
        onLocalNoSolution: QpNoSolutionListener = QpNoSolutionListener.IGNORE,
        onPairwiseNoSolution: QpNoSolutionListener = QpNoSolutionListener.IGNORE,
    ): GurobiSolver {
        val shared = sharedEnvs.get(simulationEnvironment) {
            SharedEnv(createEnv()).also { shared ->
                simulationEnvironment.simulation.addOutputMonitor(
                    object : OutputMonitor<T, P> {
                        override fun finished(environment: Environment<T, P>, time: Time, step: Long) = shared.dispose()
                    },
                )
            }
        }
        return GurobiSolver(settings, shared.env, onLocalNoSolution, onPairwiseNoSolution)
            .also { shared.solvers += it }
    }
}
