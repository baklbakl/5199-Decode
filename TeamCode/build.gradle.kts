import java.util.regex.Pattern

//
// build.gradle in TeamCode
//
// Most of the definitions for building your module reside in a common, shared
// file "build.common.gradle". Being factored in this way makes it easier to
// integrate updates to the FTC into your code. If you really need to customize
// the build definitions, you can place those customizations in this file, but
// please think carefully as to whether such customizations are really necessary
// before doing so.


// Custom definitions may go here

plugins {
    id("com.android.application")
    kotlin("android")
}

java {
    toolchain {
        languageVersion = JavaLanguageVersion.of(8)
    }
}

android {

    namespace = "net.polarbub.ftc"

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
        minSdk = (properties["androidsdk.minVersion"] as String?)?.toInt()
        compileSdk = (properties["androidsdk.compileVersion"] as String?)?.toInt()

        /**
         * We keep the versionCode and versionName of robot controller applications in sync with
         * the master information published in the AndroidManifest.xml file of the FtcRobotController
         * module. This helps avoid confusion that might arise from having multiple versions of
         * a robot controller app simultaneously installed on a robot controller device.
         *
         * We accomplish this with the help of a funky little Groovy script that maintains that
         * correspondence automatically.
         *
         * @see <a href="http://developer.android.com/tools/building/configuring-gradle.html">Configure Your Build</a>
         * @see <a href="http://developer.android.com/tools/publishing/versioning.html">Versioning Your App</a>
         */
        var manifestFile = project(":FtcRobotController").file("src/main/AndroidManifest.xml");
        var manifestText = manifestFile.readText()
        //
        var vCodePattern = Pattern.compile("versionCode=\"(\\d+(\\.\\d+)*)\"")
        var matcher = vCodePattern.matcher(manifestText)
        matcher.find()
        var vCode = Integer.parseInt(matcher.group(1))
        //
        var vNamePattern = Pattern.compile("versionName=\"(.*)\"")
        matcher = vNamePattern.matcher(manifestText);
        matcher.find()
        var vName = matcher.group(1)
        //
        versionCode = vCode
        versionName = vName
    }

    // http://google.github.io/android-gradle-dsl/current/com.android.build.gradle.internal.dsl.BuildType.html
    buildTypes {
        release {
            signingConfig = signingConfigs["release"]

            ndk {
                abiFilters.add("armeabi-v7a")
                abiFilters.add("arm64-v8a")
            }
        }

       debug {
           isDebuggable = true
           isJniDebuggable = true

           ndk {
               abiFilters.add("armeabi-v7a")
               abiFilters.add("arm64-v8a")
           }
       }
    }

//    compileOptions {
//        sourceCompatibility(JavaVersion.VERSION_1_8)
//        targetCompatibility(JavaVersion.VERSION_1_8)
//    }

    packaging {
        jniLibs.pickFirsts.add("**/*.so")
    }

    sourceSets.getByName("main") {
//        jni.srcDirs("");
        jniLibs.srcDir(rootProject.file("libs"))
    }

    ndkVersion = "21.3.6528147"
}

dependencies {
    implementation(project(":FtcRobotController"))
    implementation(project(":Common"))

    implementation("com.acmerobotics.roadrunner:ftc:0.1.14")
    implementation("com.acmerobotics.dashboard:dashboard:0.4.16")
}
