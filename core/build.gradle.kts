import org.gradle.kotlin.dsl.withType
import org.jetbrains.kotlin.gradle.tasks.KotlinCompilationTask

plugins {
    `java-library`
    alias(libs.plugins.gitSemVer)
    alias(libs.plugins.collektive)
    alias(libs.plugins.kotlin.jvm)
    alias(libs.plugins.kotlin.qa)
    alias(libs.plugins.multiJvmTesting)
    alias(libs.plugins.publishOnCentral)
}

group = "io.github.angelacorte"

dependencies {
    // Collektive and Gurobi types appear in the public API
    api(libs.collektive)
    api(libs.collektive.stdlib)
    api(libs.gurobi)
    implementation(libs.slf4j)
}

multiJvm {
    jvmVersionForCompilation.set(21)
}

publishOnCentral {
    repoOwner.set("angelacorte")
    projectLongName.set("CAROL")
    projectDescription.set(
        "Coordinated Aggregate Robotics with Online control Lyapunov and barrier functions: " +
            "distributed ADMM control with CLF/CBF constraints, built on Collektive and Gurobi",
    )
    projectUrl.set("https://github.com/angelacorte/carol-experiments")
    scmConnection.set("scm:git:https://github.com/angelacorte/carol-experiments")
}

signing {
    val signingKey: String? by project
    val signingPassword: String? by project
    useInMemoryPgpKeys(signingKey, signingPassword)
}

publishing {
    publications {
        withType<MavenPublication>().configureEach {
            artifactId = "carol"
            pom {
                developers {
                    developer {
                        name.set("Angela Cortecchia")
                        url.set("https://github.com/angelacorte")
                    }
                }
            }
        }
    }
}

tasks.withType<KotlinCompilationTask<*>>().all {
    compilerOptions {
        allWarningsAsErrors = false
    }
}
