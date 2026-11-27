package it.unibo.collektive.alchemist.scenarios

import it.unibo.alchemist.CollektiveIncarnation
import it.unibo.alchemist.actions.RunCollektiveProgram
import it.unibo.alchemist.boundary.Loader
import it.unibo.alchemist.boundary.kotlindsl.simulation2D
import it.unibo.alchemist.boundary.launchers.DefaultLauncher
import it.unibo.alchemist.boundary.swingui.monitor.impl.SwingGUI
import it.unibo.alchemist.model.Node
import it.unibo.alchemist.model.TimeDistribution
import it.unibo.alchemist.model.actions.SpeedToTarget
import it.unibo.alchemist.model.deployments.Point
import it.unibo.alchemist.model.environments.Continuous2DEnvironment
import it.unibo.alchemist.model.implementations.reactions.MoveTarget
import it.unibo.alchemist.model.networks.ConnectWithinDistance
import it.unibo.alchemist.model.positions.Euclidean2DPosition
import it.unibo.alchemist.model.reactions.GenericReaction
import it.unibo.alchemist.model.terminators.AfterTime
import it.unibo.alchemist.model.times.DoubleTime
import it.unibo.collektive.admm.Tolerance
import it.unibo.collektive.alchemist.device.SimulationQpSettings
import it.unibo.collektive.alchemist.device.sensors.impl.LocationSensorProperty
import it.unibo.collektive.alchemist.device.sensors.impl.SolverProperty
import it.unibo.collektive.alchemist.device.sensors.impl.TimeSensorProperty

/** A robot starting at ([x], [y]) that chases the target with id [targetId]. */
data class RobotSpec(val x: Double, val y: Double, val targetId: Int)

/** A target with id [id] initially placed at ([x], [y]). */
data class TargetSpec(val x: Double, val y: Double, val id: Int)

/** A static obstacle centred in ([x], [y]) with safety [radius] and extra [margin]. */
data class ObstacleSpec(val x: Double, val y: Double, val radius: Double = 3.0, val margin: Double = 2.0)

/** Moves the target [targetId] to ([x], [y]) at simulated time [time]. */
data class TargetMove(val time: Double, val x: Double, val y: Double, val targetId: Int = 1)

private const val ADMM_TOLERANCE = 1e-2

/**
 * One robot-swarm experiment: all scenarios share the same structure and differ only in these values.
 *
 * @property entrypoint `<JvmClass>.<function>` name of the Collektive program, resolved when the simulation is built
 * @property robots the robots to deploy
 * @property targets the targets to deploy
 * @property obstacles the static obstacles to deploy
 * @property targetMoves scheduled target relocations
 * @property robotMaxSpeed the `MaxSpeed` of every robot
 * @property communicationDistance the `CommunicationDistance` of every robot, not deployed when null
 * @property connectWithinDistance the range of the network model
 * @property robotSafeMargin the `SafeMargin` of every robot
 * @property timeDistribution frequency of the robot program (also deployed as `TimeDistribution`)
 * @property controlPeriodMS the `ControlPeriodMS` of every robot
 * @property qpSettings the solver settings of every robot
 */
data class Scenario(
    val entrypoint: String,
    val robots: List<RobotSpec>,
    val targets: List<TargetSpec>,
    val obstacles: List<ObstacleSpec> = emptyList(),
    val targetMoves: List<TargetMove> = emptyList(),
    val robotMaxSpeed: Double = 2.0,
    val communicationDistance: Double? = null,
    val connectWithinDistance: Double = 10.0,
    val robotSafeMargin: Double = 1.0,
    val timeDistribution: Double = 100.0,
    val controlPeriodMS: Double = 100.0,
    val qpSettings: SimulationQpSettings =
        SimulationQpSettings("qp", false, 10.0, 0.5, 2.0, Tolerance(ADMM_TOLERANCE, ADMM_TOLERANCE)),
)

/**
 * Builds the Alchemist [Loader] for this scenario.
 * With [graphics] (an effects file) a paused Swing GUI is attached; with [endTime] the run stops at that time.
 */
fun Scenario.toLoader(graphics: String? = null, endTime: Double? = null): Loader {
    val incarnation = CollektiveIncarnation<Euclidean2DPosition>()
    return simulation2D(incarnation) {
        val environment = Continuous2DEnvironment(incarnation)
        environment(environment) {
            networkModel(ConnectWithinDistance(environment, connectWithinDistance))
            targetMoves.forEach {
                environment.addReaction(MoveTarget(environment, DoubleTime(it.time), it.x, it.y, it.targetId))
            }
            endTime?.let { terminator(AfterTime(DoubleTime(it))) }
            deployments {
                robots.forEach { robot ->
                    deploy(Point(environment, robot.x, robot.y)) {
                        val node = contextOf<Node<Any?>>()
                        contents {
                            "Robot"(true)
                            "MaxSpeed"(robotMaxSpeed)
                            "SafeMargin"(robotSafeMargin)
                            "TargetID"(robot.targetId)
                            communicationDistance?.let { "CommunicationDistance"(it) }
                            "TimeDistribution"(timeDistribution)
                            "ControlPeriodMS"(controlPeriodMS)
                        }
                        withTimeDistribution(timeDistribution) {
                            program(GenericReaction(node, contextOf<TimeDistribution<Any?>>())) {
                                val reaction = contextOf<GenericReaction<Any?>>()
                                action(RunCollektiveProgram<Euclidean2DPosition>(reaction, entrypoint))
                                action(SpeedToTarget<Any?, Euclidean2DPosition>(environment, reaction))
                            }
                        }
                        capability(LocationSensorProperty(environment, node))
                        capability(TimeSensorProperty(environment, node))
                        capability(SolverProperty(environment, node, qpSettings))
                    }
                }
                targets.forEach { target ->
                    deploy(Point(environment, target.x, target.y)) { contents { "Target"(target.id) } }
                }
                obstacles.forEach { obstacle ->
                    deploy(Point(environment, obstacle.x, obstacle.y)) {
                        contents {
                            "Obstacle"(true)
                            "SafeRadius"(obstacle.radius)
                            "SafeMargin"(obstacle.margin)
                        }
                    }
                }
            }
        }
        if (graphics != null) {
            monitor(SwingGUI(environment, graphics))
            launcher(DefaultLauncher(autoStart = false))
        }
    }
}

/**
 * Runs a scenario: `<scenarioName> [graphic] [endTime=<time>]`.
 * `graphic` opens the GUI with `effects/<scenarioName>.json` (paths are relative to the working directory).
 */
fun main(args: Array<String>) {
    val name = requireNotNull(args.firstOrNull()) {
        "Usage: <scenario> [graphic] [endTime=<t>]; available: ${scenarios.keys}"
    }
    val scenario = requireNotNull(scenarios[name]) { "Unknown scenario '$name', available: ${scenarios.keys}" }
    val graphics = "effects/$name.json".takeIf { "graphic" in args }
    val endTime = args.firstOrNull { it.startsWith("endTime=") }?.substringAfter("=")?.toDouble()
    val loader = scenario.toLoader(graphics, endTime)
    loader.launcher.launch(loader)
}
