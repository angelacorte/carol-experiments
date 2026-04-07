package it.unibo.collektive.admm

import it.unibo.alchemist.collektive.device.CollektiveDevice
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.collektive.aggregate.api.Aggregate
import it.unibo.collektive.aggregate.api.exchange
import it.unibo.collektive.aggregate.api.exchanging
import it.unibo.collektive.aggregate.api.neighboring
import it.unibo.collektive.aggregate.api.sharing
import it.unibo.collektive.aggregate.toMap
import it.unibo.collektive.alchemist.device.applyControl
import it.unibo.collektive.alchemist.device.sensors.TimeSensor
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.mathutils.minus
import it.unibo.collektive.mathutils.norm
import it.unibo.collektive.mathutils.plus
import it.unibo.collektive.model.Device
import it.unibo.collektive.model.SpeedControl2D
import it.unibo.collektive.model.zeroSpeed
import it.unibo.collektive.solver.Solver
import it.unibo.collektive.solver.gurobi.QpSettings
import it.unibo.collektive.stdlib.collapse.maxBy
import it.unibo.collektive.stdlib.spreading.gossipMax
import it.unibo.collektive.stdlib.time.localDeltaTime
import it.unibo.collektive.stdlib.time.sharedTimeLeftTo
import kotlin.math.max
import kotlin.time.DurationUnit

internal data class EdgeInfo(
    val device: Device,
    val suggestedControl: SuggestedControl = SuggestedControl(),
)

/**
 * Aggregate entrypoint: runs the distributed ADMM control loop and applies the resulting velocity.
 */
context(timeSensor: TimeSensor, collektiveDevice: CollektiveDevice<Euclidean2DPosition>)
fun Aggregate<Int>.admmEntrypoint(
    device: Device,
    uNominal: DoubleArray,
    solver: Solver,
    localCLF: List<CLF>,
    localCBF: List<CBF>,
    pairwiseCBF: List<CBF>,
) {
    val deltaTime: Double = localDeltaTime(timeSensor.getTimeAsInstant()).toDouble(DurationUnit.SECONDS)
        .takeIf { it > 0.0 } ?: (1.0 / (collektiveDevice["TimeDistribution"] as Double? ?: 1.0))
    if (!solver.isLocalModelAvailable) solver.setupLocalModel(device, localCLF, localCBF)
    val result = ADMM(device, uNominal, deltaTime, solver, pairwiseCBF)
    device.applyControl(result, deltaTime)
}

context(collektiveDevice: CollektiveDevice<*>)
internal inline fun <reified ID : Comparable<ID>> Aggregate<ID>.ADMM(
    device: Device,
    uNominal: DoubleArray,
    deltaTime: Double,
    solver: Solver,
    pairwiseCBF: List<CBF>,
): SpeedControl2D =
    evolving(ControlAndDuals(device.control)) { previous ->
        val previousDuals = previous.duals
        // mi serve anche la posizione precedente presa da previous, non quella corrente - la posizione all'ultimo tempo di controllo
        val control: SpeedControl2D = solver.updateAndSolveLocal(device.copy(control = previous.control), uNominal, previousDuals, deltaTime)
        collektiveDevice["Control"] = control
        val deviceUpdated = device.copy(control = control)
        val updatedDuals: Map<ID, DualParams> = exchanging(EdgeInfo(deviceUpdated)) { field ->
            val newDuals = mutableMapOf<ID, DualParams>()
            field.map { (neighborID: ID, neighborData: EdgeInfo) ->
                val neighborInfo = neighborData.device
                val previousLocalUpdate: LocalDualUpdate = previousDuals[neighborID]?.localDualUpdate ?: LocalDualUpdate()
                when {
                    isOwnerOf(neighborID) -> {
                        if (!solver.isPairwiseModelAvailable) solver.setupPairwiseModel(deviceUpdated, neighborInfo, pairwiseCBF)
                        val (zi, zj) = solver.updateAndSolvePairwise(deviceUpdated, neighborInfo, previousLocalUpdate, deltaTime)
                        collektiveDevice["zi$neighborID"] = zi
                        collektiveDevice["zj$neighborID"] = zj
                        EdgeInfo(deviceUpdated, SuggestedControl(zi, zj))
                    }
                    else -> EdgeInfo(deviceUpdated, neighborData.suggestedControl.swap())
                }.also { edgeInfo ->
                    newDuals[neighborID] = DualParams(
                        edgeInfo.suggestedControl,
                        LocalDualUpdate(
                            previousLocalUpdate.yi + deviceUpdated.control - edgeInfo.suggestedControl.zi, // y_ij^i,t+1
                            previousLocalUpdate.yj + neighborInfo.control - edgeInfo.suggestedControl.zj, // y_ij^j,t+1
                        )
                    )
                }
            }.yielding{ newDuals.filterNot { it.key == localId } }
        }
        val controlAndDuals = ControlAndDuals(deviceUpdated.control, updatedDuals)
        val previousSuggested = previousDuals.mapValues { it.value.suggestedControl }
        val (primalResidual, dualResidual) = residualUpdate(solver.settings, controlAndDuals, previousSuggested)
        val confidence = confidence(primalResidual, dualResidual, solver.settings.tolerance)
        val scaledControl = SpeedControl2D(controlAndDuals.control.x * confidence, controlAndDuals.control.y * confidence)
        controlAndDuals.yielding { scaledControl }
//        Infos(iter, controlAndDuals).yielding { OutputControl(shouldApply, deviceUpdated.control) }
    }

fun <ID : Comparable<ID>> Aggregate<ID>.coreADMM(
    device: Device,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    deltaTime: Double,
    solver: Solver,
    pairwiseCBF: List<CBF>,
): ControlAndDuals<ID> {
    val control: SpeedControl2D = solver.updateAndSolveLocal(device, uNominal, duals, deltaTime)
    val deviceUpdated = device.copy(control = control)
    return sharing(deviceUpdated) { controls ->
        val commons: Map<ID, DualParams> = controls.neighbors.toMap()
            .filterNot { it.key == localId }
            .mapValues { (neighborId, neighbor) ->
                val incidentDuals = duals[neighborId]?.localDualUpdate ?: LocalDualUpdate()
                if (!solver.isPairwiseModelAvailable) solver.setupPairwiseModel(device, neighbor, pairwiseCBF)
                val (zi, zj) = solver.updateAndSolvePairwise(deviceUpdated, neighbor, incidentDuals, deltaTime)
                val newIncidentDuals = LocalDualUpdate(
                    incidentDuals.yi + control - zi, // y_ij^i,t+1
                    incidentDuals.yj + neighbor.control - zj, // y_ij^j,t+1
                )
                DualParams(SuggestedControl(zi, zj), newIncidentDuals)
            }
        deviceUpdated.yielding { ControlAndDuals(control, commons) }
    }
}



//        val output: ControlAndDuals<ID> = exchanging(EdgeInfo(deviceUpdated)) { controls: Field<ID, EdgeInfo> ->
//            controls.map { (neighborID: ID, neighborData: EdgeInfo) ->
//                val neighborInfo: Device = neighborData.device
//                val previousLocalUpdate: LocalDualUpdate = previousDuals[neighborID]?.localDualUpdate ?: LocalDualUpdate()
//                when {
//                    isOwnerOf(neighborID) -> {
//                        if (!solver.isPairwiseModelAvailable) solver.setupPairwiseModel(deviceUpdated, neighborInfo, pairwiseCBF)
//                        val (zi, zj) = solver.updateAndSolvePairwise(deviceUpdated, neighborInfo, previousLocalUpdate, deltaTime)
//                        collektiveDevice["zi$neighborID"] = zi
//                        collektiveDevice["zj$neighborID"] = zj
//                        EdgeInfo(deviceUpdated, SuggestedControl(zi, zj))
//                    }
//                    else -> EdgeInfo(deviceUpdated, neighborData.suggestedControl.swap())
//                }//.also { collektiveDevice["suggestedWith$neighborID"] = it.suggestedControl }
//            }.yielding {
//                val updatedDuals: Map<ID, DualParams> =
//                    controls.neighbors.sequence.associate { (neighborID, neighborData) ->
//                        val previousLocalUpdate = previousDuals[neighborID]?.localDualUpdate ?: LocalDualUpdate()
//                        val suggestedForMe = when {
//                            isOwnerOf(neighborID) -> neighborData.suggestedControl
//                            else -> neighborData.suggestedControl//.swap()
//                        }
//                        neighborID to DualParams(
//                            suggestedForMe,
//                            LocalDualUpdate(
//                                previousLocalUpdate.yi + deviceUpdated.control - suggestedForMe.zi,
//                                previousLocalUpdate.yj + neighborData.device.control - suggestedForMe.zj,
//                            ).also { collektiveDevice["localUpdate$neighborID"] = it }
//                        )
//                    }
//                ControlAndDuals(deviceUpdated.control, updatedDuals)
//            }
//        } //.also { collektiveDevice["output"] = it.control }



////    return ControlAndDuals(deviceUpdated.control, newDuals)
//    val previousSuggested = previousDuals.admmOutput.duals.toMap().mapValues { it.value.suggestedControl }
//    val (primalResidual, dualResidual) = residualUpdate(solver.settings, output, previousSuggested)
//    val nextIter = previousDuals.iteration + 1
//    val (shouldApply, iter) = when {
//        (primalResidual <= solver.settings.tolerance.primal && dualResidual <= solver.settings.tolerance.dual) ||
//            nextIter >= maxIter -> true to 0
//        else -> false to nextIter
//    }
//    val confidence = confidence(primalResidual, dualResidual, solver.settings.tolerance)
//    val scaledControl = SpeedControl2D(output.control.x * confidence, output.control.y * confidence)
////    Infos(iter, output).yielding { OutputControl(shouldApply, scaledControl) }
//
////    val updatedDuals = output.duals.mapValues { (_, dual) ->
////        dual.copy(localDualUpdate = LocalDualUpdate())
////    }
//
//    Infos(iter, output).yielding { OutputControl(shouldApply, output.control) }
//}
//
//fun <ID : Comparable<ID>> Aggregate<ID>.coreADMM(
//    device: Device,
//    uNominal: DoubleArray,
//    duals: Map<ID, DualParams>,
//    deltaTime: Double,
//    solver: Solver,
//    pairwiseCBF: List<CBF>,
//): ControlAndDuals<ID> {
//    val control: SpeedControl2D = solver.updateAndSolveLocal(device, uNominal, duals, deltaTime)
//    val deviceUpdated = device.copy(control = control)
//    return sharing(deviceUpdated) { controls ->
//        val commons: Map<ID, DualParams> = controls.neighbors.toMap()
//            .filterNot { it.key == localId }
//            .mapValues { (neighborId, neighbor) ->
//                val incidentDuals = duals[neighborId]?.localDualUpdate ?: LocalDualUpdate()
//                if (!solver.isPairwiseModelAvailable) solver.setupPairwiseModel(device, neighbor, pairwiseCBF)
//                val (zi, zj) = solver.updateAndSolvePairwise(deviceUpdated, neighbor, incidentDuals, deltaTime)
//                val newIncidentDuals = LocalDualUpdate(
//                    incidentDuals.yi + control - zi, // y_ij^i,t+1
//                    incidentDuals.yj + neighbor.control - zj, // y_ij^j,t+1
//                )
//                DualParams(SuggestedControl(zi, zj), newIncidentDuals)
//            }
//        deviceUpdated.yielding { ControlAndDuals(control, commons) }
//    }
//}
//
//context(collektiveDevice: CollektiveDevice<*>)
//fun Aggregate<Int>.coreADMMWithEdgesWithIncident(
//    device: Device,
//    uNominal: DoubleArray,
//    deltaTime: Double,
//    solver: Solver,
//    pairwiseCBF: List<CBF>,
//): ControlAndDuals<Int> {
//    val control: SpeedControl2D = solver.updateAndSolveLocal(device, uNominal, duals, deltaTime)
//    collektiveDevice["control"] = control
//    val deviceUpdated = device.copy(control = control)
//    val result: ControlAndDuals<Int> = exchanging(EdgeExchange(deviceUpdated)) { controls: Field<Int, EdgeExchange> ->
//        controls.map { (neighborID: Int, neighborData: EdgeExchange) ->
//            val oldLocalUpdate = duals[neighborID]?.localDualUpdate ?: LocalDualUpdate()
//            val neighborInfo: Device = neighborData.device
//            val previousLocalUpdate: LocalDualUpdate = duals[neighborID]?.localDualUpdate ?: LocalDualUpdate()
//            when {
//                isOwnerOf(neighborID) -> {
//                    if (!solver.isPairwiseModelAvailable) solver.setupPairwiseModel(deviceUpdated, neighborInfo, pairwiseCBF)
//                    val (zi, zj) = solver.updateAndSolvePairwise(deviceUpdated, neighborInfo, previousLocalUpdate, deltaTime)
//                    collektiveDevice["zi$neighborID"] = zi
//                    collektiveDevice["zj$neighborID"] = zj
//                    EdgeExchange(deviceUpdated, SuggestedControl(zi, zj))
//                }
//                else -> EdgeExchange(deviceUpdated, neighborData.suggestedControl.swap())
//            }
//        }.yielding {
//            this.neighbors.sequence.map { (neighborId, neighborsData) ->
//                val previousLocalUpdate = duals[neighborId]?.localDualUpdate ?: LocalDualUpdate()
//
//            }
//            DualParams(
//                edge.suggestedControl,
//                LocalDualUpdate(
//                    oldLocalUpdate.yi + deviceUpdated.control - edge.suggestedControl.zi,
//                    oldLocalUpdate.yj + neighborData.device.control - edge.suggestedControl.zj,
//                )
//            )
//            ControlAndDuals(deviceUpdated.control, updatedDuals)
//            TODO()
//        }
//    }
//    val newDuals = result.neighbors.toMap().mapValues { (neighborID: ID, neighborData: EdgeExchange) ->
//        val oldLocalUpdate = duals[neighborID]?.localDualUpdate ?: LocalDualUpdate()
//        collektiveDevice["suggested$neighborID"] = neighborData.suggestedControl
//        // I have to invert suggested controls if I'm not the owner, because mine will be the zj that my owner has evaluated
//        val suggested: SuggestedControl = when {
//            isOwnerOf(neighborID) -> neighborData.suggestedControl
//            else -> requireNotNull(neighborData.suggestedControl).swap()
//        }
//
//        collektiveDevice["suggestedOwnerCheck$neighborID"] = suggested
//        DualParams(
//            suggested,
//            LocalDualUpdate(
//                oldLocalUpdate.yi + deviceUpdated.control - suggested.zi, // y_ij^i,t+1
//                oldLocalUpdate.yj + neighborData.device.control - suggested.zj, // y_ij^j,t+1
//            ).also { collektiveDevice["localdualupadate$neighborID"] = it }
//        )
//    }
//    return ControlAndDuals(deviceUpdated.control, newDuals)
//}
//
//
////        controls.map { (neighborID, neighborData) ->
////            val neighborInfo: Device = neighborData.device
////            val neighborDuals: SuggestedControl = neighborData.suggestedControl
////            val incidentDuals = duals[neighborID]?.localDualUpdate ?: LocalDualUpdate()
////            when {
////                isOwnerOf(neighborID) -> {
////                    if (!solver.isPairwiseModelAvailable) solver.setupPairwiseModel(device, neighborInfo, pairwiseCBF)
////                    val (zi, zj) = solver.updateAndSolvePairwise(deviceUpdated, neighborInfo, incidentDuals, deltaTime)
////                    collektiveDevice["zi$neighborID"] = zi
////                    collektiveDevice["zj$neighborID"] = zj
////                    collektiveDevice["ownerof$neighborID"] = true
////                    EdgeExchange(neighborInfo, SuggestedControl(zi, zj))
////                }
////                else -> EdgeExchange(deviceUpdated, neighborDuals)
////            }
//        }

context(collektiveDevice: CollektiveDevice<*>)
fun <ID : Comparable<ID>> Aggregate<ID>.coreADMMWithEdges(
    device: Device,
    uNominal: DoubleArray,
    duals: Map<ID, DualParams>,
    deltaTime: Double,
    solver: Solver,
    pairwiseCBF: List<CBF>,
): ControlAndDuals<ID> {
    val control: SpeedControl2D = solver.updateAndSolveLocal(device, uNominal, duals, deltaTime)
    val deviceUpdated = device.copy(control = control)
    val result = exchange(EdgeInfo(deviceUpdated)) { controls ->
        controls.map { (neighborID, neighborData) ->
            val neighborInfo: Device = neighborData.device
            val incidentDuals = duals[neighborID]?.localDualUpdate ?: LocalDualUpdate()
            when {
                isOwnerOf(neighborID) -> {
                    if (!solver.isPairwiseModelAvailable) solver.setupPairwiseModel(device, neighborInfo, pairwiseCBF)
                    val (zi, zj) = solver.updateAndSolvePairwise(deviceUpdated, neighborInfo, incidentDuals, deltaTime)
                    collektiveDevice["zi$neighborID"] = zi
                    collektiveDevice["zj$neighborID"] = zj
                    EdgeInfo(deviceUpdated, SuggestedControl(zi, zj)).also {
                        collektiveDevice["suggestedFrom${localId}for$neighborID"] = it.suggestedControl
                    }
                }
                else -> EdgeInfo(deviceUpdated, neighborData.suggestedControl).also {
                    collektiveDevice["Previously?SuggestedFrom${neighborID}for$localId"] = it.suggestedControl
                }
            }
        }
    }
    val newDuals = result.all.toMap().filterNot { it.key == localId }.mapValues { (neighborID, neighborData) ->
        val oldLocalUpdate = (duals[neighborID]?.localDualUpdate ?: LocalDualUpdate())
        val suggested = when {
            isOwnerOf(neighborID) -> neighborData.suggestedControl
            else -> neighborData.suggestedControl.swap()
        }
        collektiveDevice["suggestedAfterForMe${localId}to$neighborID"] = suggested
        DualParams(
            suggested,
            LocalDualUpdate(
            oldLocalUpdate.yi + deviceUpdated.control - suggested.zi, // y_ij^i,t+1
            oldLocalUpdate.yj + neighborData.device.control - suggested.zj, // y_ij^j,t+1
            ).also { collektiveDevice["localUpdateTo$neighborID"] = it }
        )
    }
    return ControlAndDuals(deviceUpdated.control, newDuals)
}

fun <ID: Comparable<ID>> Aggregate<ID>.isOwnerOf(otherID: ID): Boolean = localId != otherID && minOf(localId, otherID) == localId

private inline fun <reified ID : Comparable<ID>> Aggregate<ID>.residualUpdate(
    settings: QpSettings,
    output: ControlAndDuals<ID>,
    previousSuggested: Map<ID, SuggestedControl>,
): Residuals {
    val currentSuggested = output.duals.filterNot { it.key == localId }.mapValues { it.value.suggestedControl }
    val primalResidualLocal = currentSuggested.maxOfOrNull { (_, v) -> (output.control - v.zi).norm() } ?: 0.0
    val primalResidualGlobal = gossipMax(primalResidualLocal)
    val dualResidualLocal = currentSuggested.maxOfOrNull { (id, v) ->
        val prev = previousSuggested[id] ?: SuggestedControl()
        settings.rhoResidual * (v.zi - prev.zi).norm()
    } ?: 0.0
    val dualResidualGlobal = gossipMax(dualResidualLocal)
    return Residuals(primalResidualGlobal, dualResidualGlobal)
}

//private fun Aggregate<Int>.neighboringResidualUpdate(settings: QpSettings, output: ControlAndDuals<Int>, previousSuggested: Map<Int, SuggestedControl>) {
//    val currentSuggested = output.duals.filterNot { it.key == localId }.mapValues { it.value.suggestedControl }
//    val primalResidualLocal = neighboring(currentSuggested).map { (id, value) ->
//        val first = currentSuggested[id]?.zi ?: zeroSpeed()
//        val second = value[localId]?.zi ?: zeroSpeed()
//        (first - second).norm()
//    }.all.maxBy { it.value }.value
//    val primalResidualGlobal = gossipMax(primalResidualLocal)
//}

private fun confidence(primalResidual: Double, dualResidual: Double, tolerance: Tolerance): Double {
    val error = max(primalResidual / tolerance.primal, dualResidual / tolerance.dual)
    return if (error <= 1.0) 1.0 else 1.0 / error
}
