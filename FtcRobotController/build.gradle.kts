import java.text.SimpleDateFormat
import java.util.*
import net.polarbub.PropertiesHelper.*;

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
    val extractedLibsImplementation = configurations["extractedLibsImplementation"]
    val extractedLibsApi = configurations["extractedLibsApi"]
    extractedLibsImplementation(project(":Inspection"))
    extractedLibsImplementation(project(":RobotServer"))
//    extractedLibsImplementation(project(":Blocks"))
//    extractedLibsImplementation(project(":OnBotJava"))

    extractedLibsApi(project(":RobotCore"))
    extractedLibsApi(project(":Hardware"))
    extractedLibsApi(project(":FtcCommon"))
    extractedLibsApi(project(":Vision"))

    val aarLibsImplementation = configurations["aarLibsImplementation"]
    val aarLibsApi = configurations["aarLibsApi"]
    val aarLibVersion = getStringProperty(project, "SDK_MAJOR_VERSION") + "." + getStringProperty(project, "SDK_MINOR_VERSION") + "." + getStringProperty(project, "SDK_POINT_VERSION")

    aarLibsImplementation("org.firstinspires.ftc:Inspection:$aarLibVersion")
    aarLibsImplementation("org.firstinspires.ftc:Blocks:$aarLibVersion")
    aarLibsImplementation("org.firstinspires.ftc:RobotServer:$aarLibVersion")
    aarLibsImplementation("org.firstinspires.ftc:OnBotJava:$aarLibVersion")

    aarLibsApi("org.firstinspires.ftc:RobotCore:$aarLibVersion")
    aarLibsApi("org.firstinspires.ftc:Hardware:$aarLibVersion")
    aarLibsApi("org.firstinspires.ftc:FtcCommon:$aarLibVersion")
    aarLibsApi("org.firstinspires.ftc:Vision:$aarLibVersion")

    implementation("androidx.appcompat:appcompat:1.7.1")
}
