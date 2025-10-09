import java.text.SimpleDateFormat
import java.util.*

plugins {
    id("com.android.library")
    id("androidDefaultSetup")
}

android {
    namespace = "com.qualcomm.onbotjava"
}

dependencies {
    implementation(project(":RobotCore"))
    implementation(project(":RobotServer"))
    implementation(project(":Blocks"))
    
    implementation(
        fileTree("libs") {
            include("*.jar")
        }
    )

    implementation("com.android.tools:r8:8.11.18")
    coreLibraryDesugaring ("com.android.tools:desugar_jdk_libs:2.1.5")
}
