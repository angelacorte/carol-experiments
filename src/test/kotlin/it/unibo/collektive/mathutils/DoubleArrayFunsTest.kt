package it.unibo.collektive.mathutils

import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Test

class DoubleArrayFunsTest {

    @Test
    fun `component-wise subtraction subtracts each component`() {
        val result = doubleArrayOf(3.0, 5.0) - doubleArrayOf(1.0, 7.0)
        assertArrayEquals(doubleArrayOf(2.0, -2.0), result)
    }

    @Test
    fun `dot product multiplies matching components and sums them`() {
        assertEquals(11.0, doubleArrayOf(1.0, 2.0) * doubleArrayOf(3.0, 4.0))
    }

    @Test
    fun `dot product rejects vectors of different dimension`() {
        assertThrows(IllegalArgumentException::class.java) {
            doubleArrayOf(1.0, 2.0) * doubleArrayOf(3.0)
        }
    }

    @Test
    fun `scalar times vector scales every component`() {
        assertArrayEquals(doubleArrayOf(2.0, -4.0), 2.0 * doubleArrayOf(1.0, -2.0))
    }

    @Test
    fun `squared norm is the sum of squared components`() {
        assertEquals(25.0, doubleArrayOf(3.0, 4.0).squaredNorm())
    }

    @Test
    fun `zeroVec creates a zero vector of the requested dimension`() {
        assertArrayEquals(doubleArrayOf(0.0, 0.0, 0.0), zeroVec(3))
    }
}
