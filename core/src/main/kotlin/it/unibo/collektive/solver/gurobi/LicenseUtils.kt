package it.unibo.collektive.solver.gurobi

import com.gurobi.gurobi.GRB
import com.gurobi.gurobi.GRBEnv
import com.gurobi.gurobi.GRBException

/**
 * Creates and starts a [GRBEnv] using the license supplied by the user at runtime.
 *
 * No license is bundled with this library. If the `GRB_WLSACCESSID`, `GRB_WLSSECRET` and `GRB_LICENSEID`
 * environment variables are all set, they are applied as Web License Service credentials; otherwise Gurobi
 * resolves the license itself (the `GRB_LICENSE_FILE` environment variable, then its default locations).
 *
 * @param logEnabled whether Gurobi should print its output.
 * @return a started Gurobi environment.
 * @throws IllegalStateException if Gurobi cannot start, typically because no valid license was found.
 */
fun defaultGurobiEnv(logEnabled: Boolean): GRBEnv {
    val env = GRBEnv(true)
    try {
        env.set(GRB.IntParam.OutputFlag, if (logEnabled) 1 else 0)
        val accessId = System.getenv("GRB_WLSACCESSID")
        val secret = System.getenv("GRB_WLSSECRET")
        val licenseId = System.getenv("GRB_LICENSEID")?.toIntOrNull()
        if (!accessId.isNullOrBlank() && !secret.isNullOrBlank() && licenseId != null) {
            env.set(GRB.StringParam.WLSAccessID, accessId)
            env.set(GRB.StringParam.WLSSecret, secret)
            env.set(GRB.IntParam.LicenseID, licenseId)
        }
        env.start()
    } catch (e: GRBException) {
        env.dispose()
        throw IllegalStateException(
            "Could not start Gurobi (error ${e.errorCode}): ${e.message}. No Gurobi license is bundled with " +
                "this library; provide your own by setting the GRB_LICENSE_FILE environment variable, placing " +
                "gurobi.lic in a Gurobi default location, setting GRB_WLSACCESSID, GRB_WLSSECRET and " +
                "GRB_LICENSEID, or passing a custom envFactory to SolverProperty.",
            e,
        )
    }
    return env
}
