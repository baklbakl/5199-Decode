plugins {
    id("java-library")
}

java {
    toolchain {
        languageVersion = JavaLanguageVersion.of(8)
    }
//    sourceCompatibility = JavaVersion.VERSION_1_8
//    targetCompatibility = JavaVersion.VERSION_1_8
}

dependencies {
    //FIX: take roadrunner in tree
    api("org.apache.commons:commons-lang3:3.17.0")
    api("com.acmerobotics.roadrunner:actions:1.0.1")
    implementation("com.acmerobotics.dashboard:core:0.4.16")
}