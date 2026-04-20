package it.unibo.collektive.solver

import it.unibo.collektive.admm.DualParams
import it.unibo.collektive.admm.LocalDualUpdate
import it.unibo.collektive.admm.SuggestedControl
import it.unibo.collektive.control.cbf.CBF
import it.unibo.collektive.control.clf.CLF
import it.unibo.collektive.model.Device
import it.unibo.collektive.model.SpeedControl2D

interface Solver {
    val isLocalModelAvailable: Boolean

    val isPairwiseModelAvailable: Boolean

    fun setupLocalModel(device: Device, localCLFs: List<CLF>, localCBFs: List<CBF>)

    fun setupPairwiseModel(device: Device, otherDevice: Device, pairwiseCBFs: List<CBF>)

    fun <ID : Comparable<ID>> updateAndSolveLocal(
        device: Device,
        uNominal: DoubleArray,
        duals: Map<ID, DualParams>,
        deltaTime: Double
    ): SpeedControl2D

    fun updateAndSolvePairwise(
        device: Device,
        otherDevice: Device,
        duals: LocalDualUpdate,
        deltaTime: Double
    ): SuggestedControl
}
