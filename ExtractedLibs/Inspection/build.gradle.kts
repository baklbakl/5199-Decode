import java.text.SimpleDateFormat
import java.util.*

plugins {
    id("com.android.library")
    id("androidDefaultSetup")
}

android {
    namespace = "org.firstinspires.inspection"
}

dependencies {
    implementation(project(":RobotCore"))
    implementation(project(":Hardware"))

    implementation("com.journeyapps:zxing-android-embedded:4.3.0")
    implementation("io.github.ehn-digital-green-development:base45:0.0.3")
}
