import java.text.SimpleDateFormat
import java.util.*

plugins {
    id("com.android.library")
    id("androidDefaultSetup")
}

android {
    namespace = "com.qualcomm.ftccommon"
}

dependencies {
    implementation(project(":RobotCore"))
    implementation(project(":RobotServer"))
    implementation(project(":Inspection"))
    implementation(project(":Hardware"))
}
