package it.unibo.collektive.device

import it.unibo.collektive.model.Device
import it.unibo.collektive.model.SpeedControl2D

/**
 * Applies a computed control to the physical or simulated robot.
 */
fun interface Actuator {
    /**
     * Applies [control] for [deltaTime] seconds and returns the updated robot state.
     */
    fun applyControl(control: SpeedControl2D, deltaTime: Double): Device
}
