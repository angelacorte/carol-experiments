package it.unibo.collektive.admm

import it.unibo.collektive.model.Coordinate
import it.unibo.collektive.model.SpeedControl2D
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/**
 * The `swap()` helpers re-express an edge state from the neighbor's perspective: a mistake here
 * silently mixes up the two endpoints of every ADMM edge, so the exchange is pinned down by tests.
 */
class AdmmStateTest {

    private val yi = Coordinate(1.0, 2.0)
    private val yj = Coordinate(-3.0, 4.0)
    private val zi = SpeedControl2D(0.5, -0.5)
    private val zj = SpeedControl2D(2.0, 1.0)

    @Test
    fun `dual update swap exchanges the endpoint variables`() {
        assertEquals(LocalDualUpdate(yj, yi), LocalDualUpdate(yi, yj).swap())
    }

    @Test
    fun `suggested control swap exchanges the endpoint controls`() {
        assertEquals(SuggestedControl(zj, zi), SuggestedControl(zi, zj).swap())
    }

    @Test
    fun `dual params swap exchanges both suggested controls and duals`() {
        val swapped = DualParams(SuggestedControl(zi, zj), LocalDualUpdate(yi, yj)).swap()
        assertEquals(DualParams(SuggestedControl(zj, zi), LocalDualUpdate(yj, yi)), swapped)
    }

    @Test
    fun `swap is an involution`() {
        val edgeState = DualParams(SuggestedControl(zi, zj), LocalDualUpdate(yi, yj))
        assertEquals(edgeState, edgeState.swap().swap())
    }

    @Test
    fun `confidence is full when residuals are within tolerance`() {
        assertEquals(1.0, confidence(0.005, 0.005, Tolerance(primal = 0.01, dual = 0.01)))
        assertEquals(1.0, confidence(0.0, 0.0, Tolerance(primal = 0.01, dual = 0.01)))
    }

    @Test
    fun `confidence shrinks inversely with the worst residual ratio`() {
        assertEquals(0.5, confidence(0.02, 0.0, Tolerance(primal = 0.01, dual = 0.01)))
        assertEquals(0.25, confidence(0.0, 0.04, Tolerance(primal = 0.01, dual = 0.01)))
        assertEquals(0.25, confidence(0.02, 0.04, Tolerance(primal = 0.01, dual = 0.01)))
    }
}
