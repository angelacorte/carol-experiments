package it.unibo.collektive.solver.gurobi

/**
 * Callback invoked when a QP solve terminates without a usable solution.
 *
 * When this happens the QP templates fall back to the previously applied control, which keeps the
 * simulation running but temporarily voids the CLF/CBF guarantees. Implementations should record
 * the event (e.g. as an exported per-node metric) so experiment data can distinguish clean runs
 * from degraded ones.
 */
fun interface QpNoSolutionListener {

    /**
     * Notifies that a solve produced no solution; [status] is the Gurobi termination status code.
     */
    fun onNoSolution(status: Int)

    /**
     * Listener implementations that ignore the event.
     */
    companion object {
        /**
         * No-op listener used when no reporting is needed (e.g. standalone tests).
         */
        val IGNORE: QpNoSolutionListener = QpNoSolutionListener { }
    }
}
