package it.unibo.collektive.alchemist.device

import it.unibo.collektive.alchemist.device.sensors.EnvironmentVariables

/*
 * Typed readers for the scenario molecules shared by every experiment.
 *
 * Reading molecules through these accessors (instead of raw `device["Name"] as Type` casts) makes a
 * YAML mistake fail immediately with a message naming the molecule, rather than silently running
 * the simulation with a fallback value or crashing later with an anonymous ClassCastException.
 */

/**
 * Control period of the ADMM loop, in milliseconds, read from the `ControlPeriodMS` molecule.
 */
val EnvironmentVariables.controlPeriodMillis: Double get() = requiredDouble("ControlPeriodMS")

/**
 * Maximum communication distance between robots, read from the `CommunicationDistance` molecule.
 */
val EnvironmentVariables.communicationDistance: Double get() = requiredDouble("CommunicationDistance")

/**
 * Identifier of the target assigned to this robot, read from the `TargetID` molecule.
 */
val EnvironmentVariables.targetId: Number get() = requiredNumber("TargetID")

/**
 * Reads the numeric concentration of [molecule] as a [Double], accepting any [Number] the YAML
 * loader produced (integers included).
 *
 * @throws IllegalStateException if the molecule is not defined for this node.
 * @throws IllegalArgumentException if the concentration is not numeric.
 */
fun EnvironmentVariables.requiredDouble(molecule: String): Double = requiredNumber(molecule).toDouble()

/**
 * Reads the numeric concentration of [molecule], failing loudly when it is missing or non-numeric.
 *
 * @throws IllegalStateException if the molecule is not defined for this node.
 * @throws IllegalArgumentException if the concentration is not numeric.
 */
fun EnvironmentVariables.requiredNumber(molecule: String): Number {
    check(isDefined(molecule)) {
        "Molecule '$molecule' is not defined for this node: check the deployment contents in the scenario YAML"
    }
    val concentration = get<Any>(molecule)
    require(concentration is Number) {
        "Molecule '$molecule' should be numeric, " +
            "but its concentration is '$concentration' (${concentration::class.simpleName})"
    }
    return concentration
}
