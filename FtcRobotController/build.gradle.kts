import java.text.SimpleDateFormat
import java.util.*
import org.ducktapeandaprayer.PropertiesHelper.*;

plugins {
    id("com.android.library")
    id("androidDefaultSetup")
    id("FTCLibFlavor")
}

android {
    namespace = "com.qualcomm.ftcrobotcontroller"

    defaultConfig {
        buildConfigField("String", "APP_BUILD_TIME", "\"" + SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ", Locale.ROOT).format(Date()) + "\"")
    }
}

dependencies {
    val implDeps = listOf("Inspection", "RobotServer")
    val aarLibVersion = getStringProperty(project, "aarLibVersion")
    val aarLibPrefix = getStringProperty(project, "aarLibPrefix")
    for(dep: String in implDeps) {
        configurations["extractedLibsImplementation"](project(":$dep"))
        configurations["aarLibsImplementation"]("$aarLibPrefix:$dep:$aarLibVersion")
    }

    val aarLibOnlyImplDeps = listOf("OnBotJava", "Blocks")
    for(dep: String in aarLibOnlyImplDeps) {
        configurations["aarLibsImplementation"]("$aarLibPrefix:$dep:$aarLibVersion")
    }

    val apiDeps = listOf("RobotCore", "Hardware", "FtcCommon", "Vision")
    for(dep: String in apiDeps) {
        configurations["extractedLibsApi"](project(":$dep"))
        configurations["aarLibsApi"]("$aarLibPrefix:$dep:$aarLibVersion")
    }

    implementation("androidx.appcompat:appcompat:1.7.1")
}
