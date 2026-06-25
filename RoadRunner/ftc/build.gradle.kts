import com.android.build.gradle.internal.packaging.defaultExcludes
import com.android.build.gradle.internal.tasks.factory.dependsOn
import com.github.gradle.node.yarn.task.YarnTask
import org.ducktapeandaprayer.PropertiesHelper.getStringProperty

plugins {
    id("com.github.node-gradle.node") version "7.1.0"
    id("com.android.library")
//    id("kotlin-android")
    id("androidDefaultSetup")
    id("FTCLibFlavor")
    id("cleanwebassets")
}

android {
    namespace = "com.acmerobotics.roadrunner"

    testOptions {
        unitTests.all {
            it.useJUnitPlatform()
            it.ignoreFailures = true
        }
    }

    enableKotlin = true
}

val webDir = file("${project.projectDir}/web")

node {
    version = "18.12.1"
    download = true
    nodeProjectDir = webDir
}

//This is in a plugin because for some reason it crashes in kotlin, but doesn't in java. See CleanWebAssetsPlugin
//tasks.register<Delete>("cleanWebAssets") {
//    fileTree("${android.sourceSets["main"].assets.srcDirs.first()}/web/").visit {
//        delete(this.file)
//    }
//}
//
//tasks.named("clean") {
//    dependsOn("cleanWebAssets")
//}

val yarn_buildTask = tasks.register<YarnTask>("yarn_build") {
    environment.put("VITE_APP_VERSION", getStringProperty(project, "roadRunnerVersion"))
    yarnCommand.set(listOf("build"))

    for(file in webDir.listFiles()) {
        if(file.name != "dist" && file.name != ".gitignore") {
            if(file.isDirectory) {
                inputs.dir(file)
            } else {
                inputs.file(file)
            }
        }

        if(file.name == "dist" && file.isDirectory) {
            outputs.dirs(file)
        }
    }

    workingDir.set(webDir)
    outputs.upToDateWhen {
        true
    }
}
tasks.preBuild.dependsOn(yarn_buildTask)

dependencies {
    val implDeps = listOf("RobotCore", "Hardware", "FtcCommon", "RobotServer")
    val aarLibVersion = getStringProperty(project, "aarLibVersion")
    val aarLibPrefix = getStringProperty(project, "aarLibPrefix")
    for(dep in implDeps) {
        configurations["extractedLibsImplementation"](project(":$dep"))
        configurations["aarLibsImplementation"]("$aarLibPrefix:$dep:$aarLibVersion")
    }

    api(project(":RoadRunner:core"))
    api("com.acmerobotics.roadrunner:actions:" + getStringProperty(project, "roadRunnerVersion")) {
        exclude("com.acmerobotics.roadrunner", "core")
    }
    api("com.acmerobotics.dashboard:dashboard:" + getStringProperty(project, "dashboardVersion")) {
        exclude("org.firstinspires.ftc", "*")
    }

    implementation("com.fasterxml.jackson.core:jackson-databind:2.12.7")

    testImplementation("org.junit.jupiter:junit-jupiter:5.9.1")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}
