import java.text.SimpleDateFormat
import java.util.*

//
// build.gradle in FtcRobotController
//

plugins {
    id("com.android.library")
}

android {
    namespace = "com.qualcomm.ftcrobotcontroller"
    buildFeatures.buildConfig = true
    lint.targetSdk = (properties["androidsdk.compileVersion"] as String?)?.toInt()

    defaultConfig {
        minSdk = (properties["androidsdk.minVersion"] as String?)?.toInt()
        compileSdk = (properties["androidsdk.compileVersion"] as String?)?.toInt()
        buildConfigField("String", "APP_BUILD_TIME", "\"" + SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ", Locale.ROOT).format(Date()) + "\"")
    }

    compileOptions {
        sourceCompatibility(JavaVersion.VERSION_1_8)
        targetCompatibility(JavaVersion.VERSION_1_8)
    }
}

dependencies {
    implementation("org.firstinspires.ftc:Inspection:" + properties["ftclibsVersion"])
    implementation("org.firstinspires.ftc:Blocks:" + properties["ftclibsVersion"])
    implementation("org.firstinspires.ftc:RobotServer:" + properties["ftclibsVersion"])
    implementation("org.firstinspires.ftc:OnBotJava:" + properties["ftclibsVersion"])

    api("org.firstinspires.ftc:RobotCore:" + properties["ftclibsVersion"])
    api("org.firstinspires.ftc:Hardware:" + properties["ftclibsVersion"])
    api("org.firstinspires.ftc:FtcCommon:" + properties["ftclibsVersion"])
    api("org.firstinspires.ftc:Vision:" + properties["ftclibsVersion"])

    implementation("androidx.appcompat:appcompat:1.2.0")
//    implementation("androidx.preference:preference:1.2.1")
}
