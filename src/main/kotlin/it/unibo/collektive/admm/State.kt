package it.unibo.collektive.admm

import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.Vector2D
import it.unibo.collektive.model.initVector2D
import it.unibo.collektive.model.zeroSpeed

/** Control paired with residuals. */
data class ControlAndResiduals(val control: SpeedControl2D, val residuals: Residuals)

/** Control decision and dual map for the current node. */
data class ControlAndDuals<ID : Comparable<ID>>(
    val control: SpeedControl2D,
    val duals: Map<ID, DualParams> = emptyMap(),
)

/** Local dual bundle for a neighbor edge. */
data class DualParams(
    val suggestedControl: SuggestedControl = SuggestedControl(),
    val incidentDuals: IncidentDuals = IncidentDuals(),
) {
    override fun toString(): String = "DualParams(suggestedControl=$suggestedControl, \n incidentDuals=$incidentDuals)"
}

/** Dual variables associated to an edge (stored from the local and neighbor perspective). */
data class IncidentDuals(val yi: Vector2D = initVector2D(), val yj: Vector2D = initVector2D()) {
    override fun toString(): String = "IncidentDuals(yi=(${yi.x}, ${yi.y}), yj=(${yj.x}, ${yj.y})"
}

/** Suggested consensus controls for the two edge endpoints. */
data class SuggestedControl(val zi: SpeedControl2D = zeroSpeed(), val zj: SpeedControl2D = zeroSpeed()) {
    override fun toString(): String = "SuggestedControl(zi=(${zi.x}, ${zi.y}), zj=(${zj.x}, ${zj.y})"
}

/** Primal/dual residuals for stopping criteria. */
data class Residuals(val primalResidual: Double, val dualResidual: Double)

/** Absolute and relative tolerances used to stop ADMM iterations. */
data class Tolerance(val primal: Double, val dual: Double)

