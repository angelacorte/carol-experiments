@file:Suppress("MatchingDeclarationName")

package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBLinExpr
import com.gurobi.gurobi.GRBModel
import com.gurobi.gurobi.GRBQuadExpr
import com.gurobi.gurobi.GRBVar
import it.unibo.collektive.control.ControlFunction
import it.unibo.collektive.mathutils.zeroVec
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths

/**
 * Wraps an array of Gurobi decision variables as a single vector.
 *
 * Provides dimension-independent access and is the unit of currency passed between
 * [ControlFunction.install] and the template's objective builder.
 */
class GRBVector(val vars: Array<GRBVar>) {
    val dimensions: Int get() = vars.size
    operator fun get(index: Int): GRBVar = vars[index]
}

/**
 * Adds [dimension] continuous scalar variables to the model, each bounded in
 * `[lowerBound, upperBound]`, and wraps them in a [GRBVector].
 */
fun GRBModel.addVecVar(dimension: Int, lowerBound: Double, upperBound: Double, name: String): GRBVector =
    GRBVector(Array(dimension) { i -> addVar(lowerBound, upperBound, 0.0, GRB.CONTINUOUS, "$name[$i]") })

/**
 * Appends `ρ·‖u − a‖²` to this expression in expanded form:
 * `ρ·Σᵢ (uᵢ² − 2aᵢuᵢ + aᵢ²)`.
 *
 * @param u   decision-variable vector
 * @param a   constant vector (same length as [u])
 * @param rho non-negative weight
 */
fun GRBQuadExpr.addRhoNorm2Sq(u: GRBVector, a: DoubleArray, rho: Double = 1.0) {
    require(u.vars.size == a.size) { "u and a must have the same length" }
    for (i in u.vars.indices) {
        addTerm(rho, u[i], u[i]) // ρ·uᵢ²
        addTerm(-2.0 * rho * a[i], u[i]) // −2ρaᵢuᵢ
        addConstant(rho * a[i] * a[i]) // ρaᵢ²
    }
}

/**
 * Constructs the linear expression `multiplier · vectorᵀ · u`.
 *
 * This is the algebraic bridge between a gradient `∇h(p)` and the QP constraint
 * `∇h(p)ᵀ u ≥ …` under first-order dynamics `ṗ = u`.
 *
 * @receiver decision-variable vector `u`
 * @param vector constant coefficient vector (must have the same length as [this])
 * @param multiplier scalar pre-multiplier (default 1.0)
 */
fun GRBVector.toLinExpr(vector: DoubleArray, multiplier: Double = 1.0): GRBLinExpr {
    require(vector.size == dimensions) { "Dimension mismatch: |v|=${vector.size}, |u|=$dimensions" }
    return GRBLinExpr().also { expr ->
        for (i in vector.indices) expr.addTerm(multiplier * vector[i], this[i])
    }
}

/**
 * Constructs the quadratic expression `coefficient · ‖u‖²` (i.e. `‖u − 0‖²` scaled).
 */
fun GRBVector.toQuadExpr(coefficient: Double = 1.0): GRBQuadExpr =
    GRBQuadExpr().also { it.addRhoNorm2Sq(this, zeroVec(dimensions), coefficient) }

private val DEFAULT_LICENSE_PATH: Path =
    Paths.get(System.getProperty("user.home"), "Library", "gurobi", "gurobi.lic")

/**
 * Resolves the Gurobi license file path.
 *
 * Search order:
 * 1. `GRB_LICENSE_FILE` environment variable
 * 2. `GRB_LICENSE_FILE` JVM system property
 * 3. Default macOS location `~/Library/gurobi/gurobi.lic`
 */
private fun resolveLicensePath(): Path? = sequenceOf(
    System.getenv("GRB_LICENSE_FILE"),
    System.getProperty("GRB_LICENSE_FILE"),
).filterNotNull()
    .filter { it.isNotBlank() }
    .map { Paths.get(it) }
    .plusElement(DEFAULT_LICENSE_PATH)
    .firstOrNull { Files.exists(it) }

/**
 * Sets the `GRB_LICENSE_FILE` system property from the first discovered license file.
 *
 * @throws IllegalStateException if no license file can be found
 */
fun setLicense() {
    val license = resolveLicensePath()
        ?: error(
            "Gurobi license not found. Set GRB_LICENSE_FILE as an environment variable " +
                "or JVM property, or place the license at '$DEFAULT_LICENSE_PATH'.",
        )
    System.setProperty("GRB_LICENSE_FILE", license.toString())
}

/** Generates prefixed constraint names for all QP sub-problems. */
object ConstraintNames {
    fun collision(edgeId: String) = "${prefix()}_collision_$edgeId"
    fun comm(edgeId: String) = "${prefix()}_comm_$edgeId"
    fun obstacle(id: String) = "${prefix()}_obstacle_$id"
    fun clf(id: String) = "${prefix()}_clf_$id"
    fun slack(id: String) = "${prefix()}_slack_$id"

    private fun prefix(): String = QpSettings().constraintPrefix
}
