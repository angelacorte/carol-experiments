package it.unibo.collektive.alchemist.device

import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class ScenarioParametersTest {

    private class FakeEnvironmentVariables(private val values: Map<String, Any>) : EnvironmentVariables {
        @Suppress("UNCHECKED_CAST")
        override fun <T> get(name: String): T = values.getValue(name) as T

        @Suppress("UNCHECKED_CAST")
        override fun <T> getOrNull(name: String): T? = values[name] as T?

        @Suppress("UNCHECKED_CAST")
        override fun <T> getOrDefault(name: String, default: T): T = values[name] as T? ?: default

        override fun isDefined(name: String): Boolean = name in values

        override fun <T> set(name: String, value: T): T = value
    }

    @Test
    fun `typed accessors read the scenario molecules`() {
        val env = FakeEnvironmentVariables(
            mapOf("ControlPeriodMS" to 250.0, "CommunicationDistance" to 9.0, "TargetID" to 2),
        )
        assertEquals(250.0, env.controlPeriodMillis)
        assertEquals(9.0, env.communicationDistance)
        assertEquals(2, env.targetId)
    }

    @Test
    fun `integer concentrations are accepted where doubles are expected`() {
        val env = FakeEnvironmentVariables(mapOf("ControlPeriodMS" to 100))
        assertEquals(100.0, env.controlPeriodMillis)
    }

    @Test
    fun `missing molecules fail loudly naming the molecule`() {
        val env = FakeEnvironmentVariables(emptyMap())
        val failure = assertThrows(IllegalStateException::class.java) { env.controlPeriodMillis }
        assertTrue("ControlPeriodMS" in failure.message.orEmpty()) {
            "The error should name the missing molecule, was: ${failure.message}"
        }
    }

    @Test
    fun `non-numeric concentrations fail loudly naming the molecule`() {
        val env = FakeEnvironmentVariables(mapOf("CommunicationDistance" to "nine"))
        val failure = assertThrows(IllegalArgumentException::class.java) { env.communicationDistance }
        assertTrue("CommunicationDistance" in failure.message.orEmpty()) {
            "The error should name the mistyped molecule, was: ${failure.message}"
        }
    }
}
