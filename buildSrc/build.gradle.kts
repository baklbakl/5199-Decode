plugins {
    `java-gradle-plugin`
//    `kotlin-dsl`
}

gradlePlugin {
    plugins {
        create("java8plugin") {
            id = name
            implementationClass = "org.ducktapeandaprayer.Java8Plugin"
        }

        create("androidDefaultSetup") {
            id = name
            implementationClass = "org.ducktapeandaprayer.AndroidDefaultSetupPlugin"
        }

        create("FTCLibFlavor") {
            id = name
            implementationClass = "org.ducktapeandaprayer.FTCLibFlavorPlugin"
        }

        create("stubPlugin") {
            id = name
            implementationClass = "org.ducktapeandaprayer.StubPlugin"
        }

        create("repositories") {
            id = name
            implementationClass = "org.ducktapeandaprayer.RepositoriesPlugin"
        }
    }
}

repositories {
    google()
    gradlePluginPortal()
}

dependencies {
    api("com.android.tools.build:gradle:9.2.1")
}