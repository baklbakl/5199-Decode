import java.text.SimpleDateFormat
import java.util.*

plugins {
    id("com.android.library")
    id("androidDefaultSetup")
}

android {
    namespace = "com.qualcomm.vision"
}

dependencies {
    implementation(project(":RobotCore"))

    api("org.openftc:easyopencv:1.7.3")
    api("org.openftc:apriltag:2.1.0")
}
