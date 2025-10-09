plugins {
    id("java-library")
    id("java8plugin")
    id("repositories")
}

dependencies {
    //FIX: take roadrunner in tree
    api("org.apache.commons:commons-lang3:3.19.0")
    api("com.acmerobotics.roadrunner:actions:1.0.1")
    implementation("com.acmerobotics.dashboard:core:0.4.17")
}