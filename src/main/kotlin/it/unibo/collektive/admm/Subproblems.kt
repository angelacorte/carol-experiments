//package it.unibo.collektive.admm
//
//import it.unibo.collektive.control.ControlFunctionContext
//import it.unibo.collektive.control.cbf.CBF
//import it.unibo.collektive.control.clf.CLF
//import it.unibo.collektive.model.Robot
//import it.unibo.collektive.model.SpeedControl2D
//import it.unibo.collektive.solver.gurobi.LocalQP
//import it.unibo.collektive.solver.gurobi.PairwiseQP
//import it.unibo.collektive.solver.gurobi.QpSettings
//
///**
// * Solves the local ADMM QP for the agent identified by [selfId] inside [simulationKey].
// *
// * On the first call for a given [selfId], a [LocalQP] is created via
// * [LocalQP.create] (which adds all variables and constraints once).
// * On every subsequent call the cached template is reused: only numerical parameters
// * (bounds, RHS, coefficients, objective) are refreshed before calling [GRBModel.optimize].
// *
// * @param simulationKey stable identity of the owning Alchemist simulation
// * @param selfId    unique, stable identifier for the calling robot (used as cache key)
// * @param robot     current robot state
// * @param uNominal  nominal control input
// * @param duals     current per-neighbour dual variables
// * @param settings  solver settings
// * @param localCLFs CLF list — must have the **same length** on every call for a given [selfId]
// * @param localCBFs CBF list — same length requirement as [localCLFs]
// */
//fun <ID : Comparable<ID>> solveLocalQP(
//    simulationKey: Any,
//    selfId: ID,
//    robot: Robot,
//    uNominal: DoubleArray,
//    duals: Map<ID, DualParams>,
////    settings: QpSettings = QpSettings(),
////    localCLFs: List<CLF>,
////    localCBFs: List<CBF> = emptyList(),
//): SpeedControl2D {
////    val template = templatesFor(simulationKey).local.computeIfAbsent(selfId as Any) {
////        LocalQP.create(robot, localCLFs, localCBFs, settings)
////    }
////    val context = ControlFunctionContext(self = robot, settings = settings)
////    return template.solve(robot, uNominal, duals, context, localCLFs, localCBFs)
//}
//
///**
// * Solves the pairwise ADMM QP for the directed edge `(selfId, neighborId)` inside [simulationKey].
// *
// * A [PairwiseQP] is created on cache miss and reused on subsequent calls for the same edge.
// * The template is indexed by the pair `(selfId, neighborId)`; new neighbours get a fresh template
// * automatically.
// *
// * @param simulationKey stable identity of the owning Alchemist simulation
// * @param selfId this agent identifier
// * @param neighborId the neighbor identifier
// * @param robot this agent state
// * @param other the neighbor state
// * @param incidentDuals current dual variables for this edge
// * @param settings solver settings
// * @param pairwiseCBFs CBF list — must have the **same length** on every call for a given edge key
// */
//fun <ID : Comparable<ID>> solvePairwiseQP(
//    simulationKey: Any,
//    selfId: ID,
//    neighborId: ID,
//    robot: Robot,
//    other: Robot,
//    incidentDuals: IncidentDuals,
//    settings: QpSettings = QpSettings(),
//    pairwiseCBFs: List<CBF> = emptyList(),
//): SuggestedControl {
//    val template = templatesFor(simulationKey).pairwise.computeIfAbsent(selfId to neighborId) {
//        PairwiseQP.create(robot, other, pairwiseCBFs, settings)
//    }
//    val context = ControlFunctionContext(self = robot, otherRobot = other, settings = settings)
//    return template.solve(robot, other, incidentDuals, context, pairwiseCBFs)
//}
