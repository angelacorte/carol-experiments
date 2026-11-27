import java.util.Locale
import org.gradle.kotlin.dsl.withType
import org.jetbrains.kotlin.gradle.tasks.KotlinCompilationTask

plugins {
    application
    alias(libs.plugins.gitSemVer)
    alias(libs.plugins.collektive)
    alias(libs.plugins.kotlin.jvm)
    alias(libs.plugins.kotlin.qa)
    alias(libs.plugins.multiJvmTesting)
}

sourceSets {
    main {
        dependencies {
            implementation(project(":core"))
            implementation(libs.bundles.alchemist)
            implementation(libs.caffeine)
            implementation(libs.bundles.collektive)
        }
    }
}

dependencies {
    testImplementation(libs.junit.jupiter)
    testRuntimeOnly(libs.junit.platform.launcher)
}

tasks.withType<Test>().configureEach {
    useJUnitPlatform()
}

multiJvm {
    jvmVersionForCompilation.set(21)
}

// Heap size estimation for batches
val maxHeap: Long? by project
val heap: Long =
    maxHeap
        ?: if (System.getProperty("os.name").lowercase().contains("linux")) {
            File("/proc/meminfo")
                .useLines { lines ->
                    lines.firstOrNull { it.startsWith("MemAvailable:") }
                        ?.trim()
                        ?.split(Regex("\\s+"))
                        ?.getOrNull(1)
                        ?.toLong()
                        ?.let { it / 1024 }
                        ?: error("MemAvailable not found in /proc/meminfo")
                }
                .toString()
                .trim()
                .toLong()
                .also { println("Detected ${it}MB RAM available.") } * 9 / 10
        } else {
            // Guess 16GB RAM of which 2 used by the OS
            14 * 1024L
        }
val taskSizeFromProject: Int? by project
val taskSize = taskSizeFromProject ?: 512
val threadCount = maxOf(1, minOf(Runtime.getRuntime().availableProcessors(), heap.toInt() / taskSize))
val alchemistGroupBatch = "Run batch simulations"
val alchemistGroupGraphic = "Run graphic simulations with Alchemist"

/*
 * This task is used to run all experiments in sequence
 */
val runAllGraphic by tasks.register<DefaultTask>("runAllGraphic") {
    group = alchemistGroupGraphic
    description = "Launches all simulations with the graphic subsystem enabled"
}
val runAllBatch by tasks.register<DefaultTask>("runAllBatch") {
    group = alchemistGroupBatch
    description = "Launches all experiments"
}

fun String.capitalizeString(): String = this.replaceFirstChar {
    if (it.isLowerCase()) {
        it.titlecase(Locale.getDefault())
    } else {
        it.toString()
    }
}

/*
 * One graphic and one batch task per scenario defined in it.unibo.collektive.alchemist.scenarios.Scenarios.kt.
 */
listOf("differentTargets", "followLeader", "followTarget", "multipleObstacles", "noObstacle").forEach { scenario ->
    fun basetask(name: String, additionalConfiguration: JavaExec.() -> Unit = {}) = tasks.register<JavaExec>(name) {
        mainClass.set("it.unibo.collektive.alchemist.scenarios.ScenarioKt")
        classpath = sourceSets["main"].runtimeClasspath
        // effects/, data/ and the analysis scripts live at the repository root
        workingDir = rootDir
        args(scenario)
        if (System.getenv("CI") == "true") {
            args("endTime=2")
        } else {
            this.additionalConfiguration()
        }
    }
    val capitalizedName = scenario.capitalizeString()
    val graphic by basetask("run${capitalizedName}Graphic") {
        group = alchemistGroupGraphic
        description = "Launches graphic simulation $scenario"
        args("graphic")
    }
    runAllGraphic.dependsOn(graphic)
    val batch by basetask("run${capitalizedName}Batch") {
        group = alchemistGroupBatch
        description = "Launches batch experiments for $capitalizedName"
        maxHeapSize = "${minOf(heap.toInt(), Runtime.getRuntime().availableProcessors() * taskSize)}m"
        rootProject.file("data").mkdirs()
    }
    runAllBatch.dependsOn(batch)
}

tasks.withType<KotlinCompilationTask<*>>().all {
    compilerOptions {
        allWarningsAsErrors = false
    }
}
