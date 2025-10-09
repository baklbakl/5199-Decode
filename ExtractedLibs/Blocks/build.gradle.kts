plugins {
    id("com.android.library")
    id("androidDefaultSetup")
}

android {
    namespace = "com.qualcomm.blocks"
}

dependencies {
    implementation(project(":RobotCore"))
    implementation(project(":RobotServer"))
    implementation(project(":FtcCommon"))
    implementation(project(":Hardware"))
    implementation(project(":Vision"))
}


