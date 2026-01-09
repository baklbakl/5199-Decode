import org.ducktapeandaprayer.PropertiesHelper.getStringProperty

buildscript {
    repositories {
        google()
        gradlePluginPortal()
    }
}

//Properties go in here if they are dynamic or are variables that are unlikely to be changed by the user
project.ext["aarLibVersion"] = getStringProperty(project, "SDK_MAJOR_VERSION") + "." + getStringProperty(project, "SDK_MINOR_VERSION") + "." + getStringProperty(project, "SDK_POINT_VERSION");
project.ext["aarLibPrefix"] = "org.firstinspires.ftc"