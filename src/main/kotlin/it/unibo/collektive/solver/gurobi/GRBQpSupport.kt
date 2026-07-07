package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import it.unibo.collektive.model.SpeedControl2D

/**
 * Sets the lower bound, upper bound, and warm-start value for every scalar component of [this],
 * mirroring [maxSpeed] as the symmetric bound and [warmStart] as the per-component start value.
 *
 * This is the bounds/warm-start step shared by [LocalQP.updateAndSolve] and
 * [PairwiseQP.updateAndSolve]: both re-apply device-dependent bounds and a warm start to their
 * decision vector(s) before refreshing constraints and solving.
 */
internal fun GRBVector.applyBoundsAndWarmStart(maxSpeed: Double, warmStart: DoubleArray) {
    for (i in variables.indices) {
        this[i].set(GRB.DoubleAttr.LB, -maxSpeed)
        this[i].set(GRB.DoubleAttr.UB, maxSpeed)
        this[i].set(GRB.DoubleAttr.Start, warmStart[i])
    }
}

/**
 * Adds the quadratic slack penalty declared by each installed constraint that has one, skipping
 * constraints installed with [it.unibo.collektive.control.dsl.SlackPolicy.None] or without a weight.
 */
internal fun GRBQuadExpr.addSlackPenalties(constraints: List<InstalledControlConstraint>) {
    constraints.forEach { constraint ->
        val slack = constraint.slack ?: return@forEach
        val weight = constraint.slackWeight ?: return@forEach
        addTerm(weight, slack, slack)
    }
}

/**
 * Updates the model, optimizes it, and recovers diagnostics when the solver does not end cleanly.
 *
 * On [GRB.INFEASIBLE] the IIS is computed and written to `logging/[iisFileName]`, and every
 * constraint participating in it is printed. On [GRB.INF_OR_UNBD] Gurobi is asked to disable dual
 * reductions and re-solve once, so the true status (infeasible vs. unbounded) can be determined.
 */
internal fun GRBModel.optimizeWithDiagnostics(iisFileName: String) {
    update()
    optimize()
    if (get(GRB.IntAttr.Status) == GRB.INFEASIBLE) {
        writeIIS(iisFileName)
        constrs.filter { it.get(GRB.IntAttr.IISConstr) == 1 }
            .forEach { println("IIS constraint: ${it.get(GRB.StringAttr.ConstrName)}") }
    }
    if (get(GRB.IntAttr.Status) == GRB.INF_OR_UNBD) {
        set(GRB.IntParam.DualReductions, 0)
        reset()
        optimize()
    }
}

/**
 * Reads the 2D speed encoded by the first two scalar components of [this] after a successful solve.
 */
internal fun GRBVector.readSpeedControl(): SpeedControl2D =
    SpeedControl2D(this[0].get(GRB.DoubleAttr.X), this[1].get(GRB.DoubleAttr.X))
