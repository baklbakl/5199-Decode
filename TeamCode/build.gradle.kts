import org.ducktapeandaprayer.PropertiesHelper.*

plugins {
    id("com.android.application")
    kotlin("android")
    id("androidDefaultSetup")
    id("FTCLibFlavor")
//    id("stubPlugin")
}

android {
    namespace = "org.ducktapeandaprayer.ftc"
    buildFeatures.buildConfig = true

    packaging {
        jniLibs.useLegacyPackaging = true
    }

    signingConfigs {
        create("release") {
            var apkStoreFile = System.getenv("APK_SIGNING_STORE_FILE")
            if (apkStoreFile != null) {
                keyAlias = System.getenv("APK_SIGNING_KEY_ALIAS")
                keyPassword = System.getenv("APK_SIGNING_KEY_PASSWORD")
                storeFile = file(System.getenv("APK_SIGNING_STORE_FILE"))
                storePassword = System.getenv("APK_SIGNING_STORE_PASSWORD")
            } else {
                keyAlias = "androiddebugkey"
                keyPassword = "android"
                storeFile = rootProject.file("libs/ftc.debug.keystore")
                storePassword = "android"
            }
        }

        getByName("debug") {
            keyAlias = "androiddebugkey"
            keyPassword = "android"
            storeFile = rootProject.file("libs/ftc.debug.keystore")
            storePassword = "android"
        }
    }

    defaultConfig {
        signingConfig = signingConfigs["debug"];
        applicationId = "com.qualcomm.ftcrobotcontroller"

        //noinspection ExpiredTargetSdkVersion
        versionCode = getIntProperty(project, "SDK_VERSION_CODE");
        val pointVersion = getStringProperty(project, "SDK_POINT_VERSION")

        versionName = getStringProperty(project, "SDK_MAJOR_VERSION") + "." + getStringProperty(
            project,
            "SDK_MINOR_VERSION"
        ) + (if (pointVersion != "0") {
            ".$pointVersion"
        } else {
            ""
        })
    }

    // http://google.github.io/android-gradle-dsl/current/com.android.build.gradle.internal.dsl.BuildType.html
    buildTypes {
        release {
            signingConfig = signingConfigs["release"]
        }
    }

    sourceSets.getByName("main") {
        jniLibs.srcDir(rootProject.file("libs"))
    }
}

dependencies {
    implementation(project(":FtcRobotController"))
    implementation(project(":Common"))

    implementation(project(":RoadRunner:ftc"))

    implementation("com.acmerobotics.dashboard:dashboard:"  + getStringProperty(project, "dashboardVersion")) {
        exclude("org.firstinspires.ftc", "*")
    }
}
