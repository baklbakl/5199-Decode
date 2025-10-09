plugins {
    `java-gradle-plugin`
//    `kotlin-dsl`
}

gradlePlugin {
    plugins {
        create("java8plugin") {
            id = name
            implementationClass = "net.polarbub.Java8Plugin"
        }

        create("androidDefaultSetup") {
            id = name
            implementationClass = "net.polarbub.AndroidDefaultSetupPlugin"
        }

        create("FTCLibFlavor") {
            id = name
            implementationClass = "net.polarbub.FTCLibFlavorPlugin"
        }

        create("stubPlugin") {
            id = name
            implementationClass = "net.polarbub.StubPlugin"
        }

        create("repositories") {
            id = name
            implementationClass = "net.polarbub.RepositoriesPlugin"
        }
    }
}

repositories {
    google()
    gradlePluginPortal()
}

dependencies {
    api("com.android.tools.build:gradle:8.12.3")
    api("org.jetbrains.kotlin:kotlin-gradle-plugin:2.1.10")
}