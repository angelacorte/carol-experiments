plugins {
    alias(libs.plugins.gitSemVer) apply false
    alias(libs.plugins.collektive) apply false
    alias(libs.plugins.kotlin.jvm) apply false
    alias(libs.plugins.kotlin.qa) apply false
    alias(libs.plugins.multiJvmTesting) apply false
    alias(libs.plugins.publishOnCentral) apply false
    alias(libs.plugins.taskTree)
}

subprojects {
    repositories {
        mavenCentral()
    }
}
