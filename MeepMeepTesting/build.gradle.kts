plugins {
    id("java")
    kotlin("jvm")
    id("java8plugin")
    id("repositories")
//    id("stubPlugin")
}

val mainClassPath: String = "net.polarbub.ftc.MeepMeepTesting"

tasks.named<Jar>("jar") {
    this.manifest.attributes["Main-Class"] = mainClassPath
    this.archiveClassifier.set("no-libs")
}

tasks.named("assemble") {
    finalizedBy("fatJar")
}

tasks.register<Jar>("fatJar") {
    this.manifest.attributes["Main-Class"] = mainClassPath
    from(sourceSets.main.get().output)

    dependsOn(configurations.runtimeClasspath)
    from({
        duplicatesStrategy = DuplicatesStrategy.EXCLUDE
        configurations.runtimeClasspath.get().filter { it.name.endsWith("jar") }.map { zipTree(it) }
    })
}

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(17))

    }

//    sourceCompatibility = JavaVersion.VERSION_17
//    targetCompatibility = JavaVersion.VERSION_17
}
//
//kotlin {
//    jvmToolchain(17)
//}

dependencies {
    implementation("com.acmerobotics.roadrunner:MeepMeep:0.1.7")
    implementation(project(":Common"))
}