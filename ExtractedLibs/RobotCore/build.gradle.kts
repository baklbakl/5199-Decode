import org.ducktapeandaprayer.PropertiesHelper.*
import java.text.SimpleDateFormat
import java.util.*

plugins {
    id("com.android.library")
    id("androidDefaultSetup")
}

android {
    namespace = "com.qualcomm.robotcore"

    defaultConfig {
        renderscriptTargetApi = getIntProperty(project, "androidsdk.compileVersion")
        renderscriptSupportModeEnabled = true
        renderscriptNdkModeEnabled = false

        buildConfigField("int", "RS_TARGET_API", getStringProperty(project, "androidsdk.compileVersion"))

        buildConfigField("String", "SDK_BUILD_TIME", "\"" + SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ", Locale.ROOT).format(Date()) + "\"")
        buildConfigField("int", "SDK_MAJOR_VERSION", (properties["SDK_MAJOR_VERSION"] as String?) ?: "")
        buildConfigField("int", "SDK_MINOR_VERSION", (properties["SDK_MINOR_VERSION"] as String?) ?: "")
        buildConfigField("int", "SDK_POINT_VERSION", (properties["SDK_POINT_VERSION"] as String?) ?: "")
    }

    externalNativeBuild {
        cmake {
            path("CMakeLists.txt")
        }
    }
}

dependencies {
    api("com.google.code.gson:gson:2.13.2")
    api("androidx.appcompat:appcompat:1.7.1")
    api("com.jakewharton.threetenabp:threetenabp:1.4.9")

    //Replacment of JARs
    api("org.danilopianini:gson-extras:3.3.0")
    //Highest version before 2.x
    api("org.slf4j:slf4j-api:1.7.36")
    //Highest version before 2.x
    api("com.gitlab.mvysny.slf4j:slf4j-handroid:1.7.30")

}
