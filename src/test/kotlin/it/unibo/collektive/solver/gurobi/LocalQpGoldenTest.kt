package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.admm.DualParams
import it.unibo.collektive.control.cbf.MaxSpeedCBF
import it.unibo.collektive.model.Device
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/**
 * Golden tests for [LocalQP]: with only a [MaxSpeedCBF] installed and no ADMM duals, the QP
 * minimizes `‖u − uNominal‖²` inside the ball `‖u‖ ≤ maxSpeed`, whose solution is the Euclidean
 * projection of the nominal control onto the ball.
 */
class LocalQpGoldenTest {

    private val precision = 1e-3

    private fun solveWith(uNominal: DoubleArray, maxSpeed: Double): DoubleArray {
        val environment = GurobiTestSupport.requireGurobi()
        val device = Device(x = 0.0, y = 0.0, safeMargin = 0.5, maxSpeed = maxSpeed)
        val qp = LocalQP.create(
            GRBModel(environment),
            device,
            localCLFs = emptyList(),
            localCBFs = listOf(MaxSpeedCBF()),
        )
        try {
            val control = qp.updateAndSolve(
                device,
                uNominal,
                duals = emptyMap<Int, DualParams>(),
                settings = GurobiTestSupport.settings,
                deltaTime = 0.1,
            )
            return doubleArrayOf(control.x, control.y)
        } finally {
            qp.dispose()
        }
    }

    @Test
    fun `nominal control within the speed limit is returned unchanged`() {
        val solution = solveWith(uNominal = doubleArrayOf(1.0, -0.5), maxSpeed = 2.0)
        assertEquals(1.0, solution[0], precision)
        assertEquals(-0.5, solution[1], precision)
    }

    @Test
    fun `nominal control beyond the speed limit is projected onto the speed ball`() {
        // ‖(3, 4)‖ = 5 > 2, so the optimum is (3, 4) · 2/5 = (1.2, 1.6).
        val solution = solveWith(uNominal = doubleArrayOf(3.0, 4.0), maxSpeed = 2.0)
        assertEquals(1.2, solution[0], precision)
        assertEquals(1.6, solution[1], precision)
    }

    @Test
    fun `zero nominal control stays zero`() {
        val solution = solveWith(uNominal = doubleArrayOf(0.0, 0.0), maxSpeed = 2.0)
        assertEquals(0.0, solution[0], precision)
        assertEquals(0.0, solution[1], precision)
    }
}
