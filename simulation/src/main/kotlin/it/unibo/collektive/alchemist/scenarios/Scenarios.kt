package it.unibo.collektive.alchemist.scenarios

private const val ENTRYPOINTS = "it.unibo.collektive.entrypoints"

private val lineOfFour = listOf(0.0, 7.0, 13.0, 20.0).map { RobotSpec(it, 0.0, 1) }

/**
 * All experiments, by name. The name selects the Gradle tasks (`run<Name>Graphic`/`run<Name>Batch`)
 * and the GUI effects file `effects/<name>.json`.
 */
val scenarios: Map<String, Scenario> = mapOf(
    "differentTargets" to Scenario(
        entrypoint = "$ENTRYPOINTS.MultipleTargetsKt.multipleTargetEntrypoint",
        robots = listOf(
            RobotSpec(0.0, 1.0, 1),
            RobotSpec(27.0, 3.0, 1),
            RobotSpec(13.0, 0.0, 2),
            RobotSpec(20.0, 2.0, 2),
        ),
        targets = listOf(TargetSpec(20.0, 20.0, 1), TargetSpec(0.0, 15.0, 2)),
        obstacles = listOf(ObstacleSpec(10.0, 10.0)),
        targetMoves = listOf(TargetMove(80.0, 15.0, 0.0), TargetMove(140.0, 0.0, 17.0)),
        robotMaxSpeed = 4.0,
    ),
    "followLeader" to Scenario(
        entrypoint = "$ENTRYPOINTS.FollowLeaderKt.followLeaderEntrypoint",
        robots = listOf(
            RobotSpec(0.0, 0.0, 1),
            RobotSpec(7.0, 0.0, 1),
            RobotSpec(5.0, -5.0, 1),
            RobotSpec(0.0, -7.0, 1),
        ),
        targets = listOf(TargetSpec(20.0, 20.0, 1)),
        obstacles = listOf(ObstacleSpec(10.0, 10.0)),
        targetMoves = listOf(
            TargetMove(60.0, 15.0, 0.0),
            TargetMove(120.0, 0.0, 0.0),
            TargetMove(180.0, 0.0, 20.0),
            TargetMove(240.0, 20.0, 0.0),
        ),
        communicationDistance = 9.0,
    ),
    "followTarget" to Scenario(
        entrypoint = "$ENTRYPOINTS.CommonTargetKt.commonTargetEntrypoint",
        robots = lineOfFour,
        targets = listOf(TargetSpec(20.0, 20.0, 1)),
        obstacles = listOf(ObstacleSpec(10.0, 10.0)),
        targetMoves = listOf(TargetMove(40.0, 15.0, 0.0), TargetMove(90.0, 0.0, 15.0)),
        communicationDistance = 9.0,
    ),
    "multipleObstacles" to Scenario(
        entrypoint = "$ENTRYPOINTS.MultipleObstaclesKt.multipleObstaclesEntrypoint",
        robots = lineOfFour,
        targets = listOf(TargetSpec(20.0, 20.0, 1)),
        obstacles = listOf(ObstacleSpec(7.8, 15.0, radius = 3.01), ObstacleSpec(17.8, 15.0, radius = 3.01)),
        communicationDistance = 9.0,
    ),
    "noObstacle" to Scenario(
        entrypoint = "$ENTRYPOINTS.NoObstacleKt.noObstacleEntrypoint",
        robots = lineOfFour.mapIndexed { index, robot -> robot.copy(targetId = if (index < 2) 1 else 2) },
        targets = listOf(TargetSpec(20.0, 20.0, 1), TargetSpec(0.0, 15.0, 2)),
        targetMoves = listOf(TargetMove(40.0, 15.0, 0.0), TargetMove(90.0, 0.0, 15.0)),
    ),
)
