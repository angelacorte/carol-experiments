package it.unibo.collektive.control.dsl

/**
 * Describes how a formula-backed control function should introduce its slack variable.
 *
 * The policy is evaluated during model installation, before the formula is compiled into Gurobi.
 * The resulting slack, if any, is then exposed in the [ControlFunctionScope] as the affine term
 * `slack` and returned through the installed
 * [it.unibo.collektive.solver.gurobi.InstalledControlConstraint].
 */
enum class SlackPolicy {
    /** Do not create a slack variable; the formula is installed as a hard constraint. */
    None,

    /** Create a slack variable only when the control function declares a finite positive slack weight. */
    Optional,

    /** Always create a slack variable and require a finite positive slack weight. */
    Required,
}
