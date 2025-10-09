import java.text.SimpleDateFormat
import java.util.*

plugins {
    id("com.android.library")
    id("androidDefaultSetup")
}

android {
    namespace = "org.firstinspires.ftc.robotcore.internal.webserver"
}

dependencies {
    implementation(project(":RobotCore"))
    implementation("org.java-websocket:Java-WebSocket:1.6.0")
}
