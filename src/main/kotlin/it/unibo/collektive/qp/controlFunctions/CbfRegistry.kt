package it.unibo.collektive.qp.controlFunctions

import com.gurobi.gurobi.GRBModel
import it.unibo.collektive.qp.config.QpSettings
import it.unibo.collektive.qp.dsl.GRBVector
import it.unibo.collektive.qp.utils.Obstacle
import it.unibo.collektive.qp.utils.Robot

/** Context passed to CBF builders. */
data class CbfContext(
    val self: Robot,
    val other: Robot? = null,
    val obstacle: Obstacle? = null,
    val communicationRange: Double? = null,
    val settings: QpSettings = QpSettings(),
)

/** Pluggable barrier builder. */
interface Cbf {
    val name: String
    fun add(model: GRBModel, uSelf: GRBVector, uOther: GRBVector?, ctx: CbfContext)
}

/** Simple registry to collect active barriers. */
object CbfRegistry {
    private val cbfs = mutableListOf<Cbf>()
    private var defaultsRegistered = false

    fun register(cbf: Cbf) {
        if (cbfs.none { it.name == cbf.name }) cbfs += cbf
    }

    fun all(): List<Cbf> {
        if (!defaultsRegistered) {
            register(ObstacleCbf)
            register(CollisionCbf)
            register(CommunicationRangeCbf)
            defaultsRegistered = true
        }
        return cbfs.toList()
    }
}

/** Apply all registered CBFs for a single-agent (local) problem. */
fun applyLocalCbfs(model: GRBModel, u: GRBVector, ctx: CbfContext) =
    CbfRegistry.all().forEach { it.add(model, u, null, ctx) }

/** Apply all registered CBFs for a pairwise problem. */
fun applyPairwiseCbfs(model: GRBModel, ui: GRBVector, uj: GRBVector, ctx: CbfContext) =
    CbfRegistry.all().forEach { it.add(model, ui, uj, ctx) }

