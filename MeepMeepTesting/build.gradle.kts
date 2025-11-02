plugins {
    id("application")
    kotlin("jvm")
    id("java8plugin")
    id("repositories")
//    id("stubPlugin")
}



val mainClassPath: String = "net.polarbub.ftc.MeepMeepTesting"

application {
    mainClass = mainClassPath
}

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

dependencies {
    implementation("com.acmerobotics.roadrunner:MeepMeep:0.1.7") {
        exclude("com.acmerobotics.roadrunner", "core")
    }
    implementation(project(":Common"))
}
