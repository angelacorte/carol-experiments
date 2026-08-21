package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.admm.LocalDualUpdate
import it.unibo.collektive.admm.SuggestedControl
import it.unibo.collektive.control.cbf.CollisionAvoidanceCBF
import it.unibo.collektive.model.Device
import it.unibo.collektive.model.SpeedControl2D
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/**
 * Golden tests for [PairwiseQP] with a hard [CollisionAvoidanceCBF] and zero duals.
 *
 * The edge QP minimizes `(ρ/2)(‖z_i − u_i‖² + ‖z_j − u_j‖²)` subject to the linearized collision
 * constraint `2(p_i − p_j)ᵀ(z_i − z_j) ≥ −(η/Δt)·h` with `h = ‖p_i − p_j‖² − d_min²`. When the
 * constraint is inactive the optimum is `(u_i, u_j)`; when active, it is the projection of
 * `(u_i, u_j)` onto the constraint halfspace in the joint decision space.
 */
class PairwiseQpGoldenTest {

    private val precision = 1e-3
    private val eta = 0.5
    private val deltaTime = 0.1

    // Devices 2 apart on the x axis with safety margin 1: h = 2² − 1² = 3, and the constraint is
    // −4(z_i.x − z_j.x) ≥ −(0.5/0.1)·3, i.e. z_i.x − z_j.x ≤ 3.75.
    private fun solveWith(selfControl: SpeedControl2D, otherControl: SpeedControl2D): SuggestedControl {
        val environment = GurobiTestSupport.requireGurobi()
        val self = Device(x = 0.0, y = 0.0, safeMargin = 1.0, control = selfControl, maxSpeed = 10.0)
        val other = Device(x = 2.0, y = 0.0, safeMargin = 1.0, control = otherControl, maxSpeed = 10.0)
        val qp = PairwiseQP.create(
            GRBModel(environment),
            self,
            other,
            pairwiseCBFs = listOf(CollisionAvoidanceCBF(eta)),
        )
        try {
            return qp.updateAndSolve(self, other, LocalDualUpdate(), GurobiTestSupport.settings, deltaTime)
        } finally {
            qp.dispose()
        }
    }

    @Test
    fun `controls that keep the pair safe are left untouched`() {
        val suggested = solveWith(SpeedControl2D(0.5, 0.3), SpeedControl2D(-0.2, 0.1))
        assertEquals(0.5, suggested.zi.x, precision)
        assertEquals(0.3, suggested.zi.y, precision)
        assertEquals(-0.2, suggested.zj.x, precision)
        assertEquals(0.1, suggested.zj.y, precision)
    }

    @Test
    fun `colliding controls are projected onto the constraint halfspace`() {
        // u_i.x − u_j.x = 5 violates the bound 3.75 by δ = 1.25; the joint projection moves each
        // endpoint by δ/2 along ∓x: z_i = (3 − 0.625, 0), z_j = (−2 + 0.625, 0).
        val suggested = solveWith(SpeedControl2D(3.0, 0.0), SpeedControl2D(-2.0, 0.0))
        assertEquals(2.375, suggested.zi.x, precision)
        assertEquals(0.0, suggested.zi.y, precision)
        assertEquals(-1.375, suggested.zj.x, precision)
        assertEquals(0.0, suggested.zj.y, precision)
        assertEquals(3.75, suggested.zi.x - suggested.zj.x, precision)
    }
}
