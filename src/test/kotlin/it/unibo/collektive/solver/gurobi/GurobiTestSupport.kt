package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import it.unibo.collektive.admm.Tolerance
import it.unibo.collektive.alchemist.device.SimulationQpSettings
import org.junit.jupiter.api.Assumptions.assumeTrue

/**
 * Shared support for tests that need a real Gurobi environment.
 *
 * Golden tests solve tiny QPs whose analytic solution is known and compare Gurobi's output against
 * it. When no usable Gurobi license is available on the machine the tests are skipped (not failed),
 * so the pure-Kotlin test suite stays runnable everywhere; CI provides a license and runs them.
 */
object GurobiTestSupport {

    /**
     * Numerical settings mirroring the values used by the simulation scenarios.
     */
    val settings: QpSettings = SimulationQpSettings(
        constraintPrefix = "qp",
        logEnabled = false,
        rhoADMM = 10.0,
        rhoResidual = 0.5,
        rhoSlack = 2.0,
        tolerance = Tolerance(primal = 1e-2, dual = 1e-2),
    )

    private val gurobiEnvironment: Result<GRBEnv> by lazy {
        runCatching {
            runCatching { setLicense() }
            GRBEnv(true).apply {
                set(GRB.IntParam.OutputFlag, 0)
                start()
            }
        }
    }

    /**
     * Returns a started [GRBEnv], or skips the calling test when Gurobi cannot be licensed here.
     */
    fun requireGurobi(): GRBEnv {
        assumeTrue(
            gurobiEnvironment.isSuccess,
            "Skipping Gurobi-backed test: ${gurobiEnvironment.exceptionOrNull()?.message}",
        )
        return gurobiEnvironment.getOrThrow()
    }
}
