package it.unibo.collektive.solver.gurobi

import it.unibo.collektive.control.ControlFunction

/**
 * Matches [installed] control functions against [current] ones by [ControlFunction.name] and calls
 * [ControlFunction.syncFrom] on each, instead of relying on list ordering.
 *
 * Used by both [LocalQP] and [PairwiseQP] to propagate dynamic providers (a moving obstacle, a
 * moving target, ...) onto control functions that were installed once and never rebuilt.
 *
 * Fails fast if the counts differ, if [current] contains duplicate names, or if some installed
 * function has no counterpart in [current] — all three would otherwise silently sync the wrong
 * provider onto the wrong installed Gurobi constraint.
 */
internal fun <T : ControlFunction> syncByName(installed: List<T>, current: List<T>, label: String) {
    require(installed.size == current.size) {
        "Expected ${installed.size} $label, got ${current.size}"
    }
    val duplicated = current.groupingBy { it.name }.eachCount().filterValues { it > 1 }.keys
    require(duplicated.isEmpty()) {
        "$label names must be unique per model instance, found duplicates: $duplicated"
    }
    val currentByName = current.associateBy { it.name }
    installed.forEach { installedFunction ->
        val update = currentByName[installedFunction.name]
            ?: error("$label: no current function named '${installedFunction.name}' to sync from")
        installedFunction.syncFrom(update)
    }
}
