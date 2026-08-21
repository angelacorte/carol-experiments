package it.unibo.collektive.mathutils

import it.unibo.collektive.model.Coordinate
import it.unibo.collektive.model.SpeedControl2D
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

class VectorFunsTest {

    @Test
    fun `speed controls sum component-wise`() {
        assertEquals(SpeedControl2D(4.0, 6.0), SpeedControl2D(1.0, 2.0) + SpeedControl2D(3.0, 4.0))
    }

    @Test
    fun `speed control division scales both components`() {
        assertEquals(SpeedControl2D(1.0, 2.0), SpeedControl2D(2.0, 4.0) / 2.0)
    }

    @Test
    fun `speed control multiplication scales both components`() {
        assertEquals(SpeedControl2D(3.0, -6.0), SpeedControl2D(1.0, -2.0) * 3.0)
    }

    @Test
    fun `vector sum and subtraction are component-wise`() {
        val left = Coordinate(1.0, 2.0)
        val right = Coordinate(3.0, 5.0)
        assertEquals(Coordinate(4.0, 7.0), left + right)
        assertEquals(Coordinate(-2.0, -3.0), left - right)
    }

    @Test
    fun `norm is the Euclidean magnitude`() {
        assertEquals(5.0, Coordinate(3.0, 4.0).norm())
    }

    @Test
    fun `nominal controller points toward the goal proportionally to the error`() {
        val control = Coordinate(2.0, 0.0).nominal(Coordinate(0.0, 0.0), controlGain = 0.5)
        assertEquals(-1.0, control.x, 0.0)
        assertEquals(0.0, control.y, 0.0)
    }

    @Test
    fun `nominal controller is zero at the goal`() {
        val control = Coordinate(1.0, 1.0).nominal(Coordinate(1.0, 1.0))
        assertEquals(0.0, control.x, 0.0)
        assertEquals(0.0, control.y, 0.0)
    }

    @Test
    fun `vector dot product and scalar product behave as expected`() {
        assertEquals(11.0, Coordinate(1.0, 2.0) * Coordinate(3.0, 4.0))
        assertEquals(Coordinate(2.0, 4.0), 2.0 * Coordinate(1.0, 2.0))
    }

    @Test
    fun `toDoubleArray preserves component order`() {
        assertArrayEquals(doubleArrayOf(1.5, -2.5), Coordinate(1.5, -2.5).toDoubleArray())
    }
}
