buildscript {
    repositories {
        google()
        gradlePluginPortal()

    }
    //FIX: Update AGP
    dependencies {
        classpath("com.android.tools.build:gradle:8.7.3")
        classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:2.1.10")

    }
}

// This is now required because aapt2 has to be downloaded from the
// google() repository beginning with version 3.2 of the Android Gradle Plugin
allprojects {
    repositories {
        mavenCentral()
        google()
        maven("https://maven.brott.dev/")
    }

    tasks.withType<JavaCompile>() {
        options.isDeprecation = true;
    }
}