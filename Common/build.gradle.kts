import org.ducktapeandaprayer.PropertiesHelper

plugins {
    id("java-library")
    id("java8plugin")
    id("repositories")
//    id("propertiesHelper")
}

dependencies {
    //FIX: take roadrunner in tree
    api("org.apache.commons:commons-lang3:3.19.0")
    api(project(":RoadRunner:core"))
    api("com.acmerobotics.roadrunner:actions:1.0.1") {
        exclude("com.acmerobotics.roadrunner", "core")
    }
    implementation("com.acmerobotics.dashboard:core:" + PropertiesHelper.getStringProperty(project, "dashboardVersion"))
}