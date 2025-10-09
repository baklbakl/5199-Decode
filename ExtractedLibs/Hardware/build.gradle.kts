plugins {
    id("com.android.library")
    id("androidDefaultSetup")
}

android {
    namespace = "com.qualcomm.hardware"
}

dependencies {
    implementation(project(":RobotCore"))
}
