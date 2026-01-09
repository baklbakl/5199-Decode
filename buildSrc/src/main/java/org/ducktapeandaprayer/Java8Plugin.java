package org.ducktapeandaprayer;

import com.android.build.api.dsl.CommonExtension;
import com.android.build.api.dsl.CompileOptions;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.JavaVersion;
import org.gradle.api.plugins.JavaPluginExtension;
import org.gradle.api.plugins.PluginManager;
import org.jetbrains.kotlin.gradle.dsl.JvmTarget;
import org.jetbrains.kotlin.gradle.dsl.KotlinAndroidExtension;
import org.jetbrains.kotlin.gradle.dsl.KotlinJvmExtension;

//See https://docs.gradle.org/current/userguide/implementing_gradle_plugins_binary.html
public class Java8Plugin implements Plugin<Project> {
    private final JavaVersion version = JavaVersion.VERSION_1_8;

    @Override
    public void apply(Project target) {
        PluginManager manager = target.getPluginManager();

        if(manager.hasPlugin("com.android.base")) {
            CommonExtension extension = target.getExtensions().findByType(CommonExtension.class);
            CompileOptions options = extension.getCompileOptions();
            options.setTargetCompatibility(version);
            options.setSourceCompatibility(version);

            if(manager.hasPlugin("org.jetbrains.kotlin.android")) {
                KotlinAndroidExtension kotlinExtension = target.getExtensions().findByType(KotlinAndroidExtension.class);
                kotlinExtension.getCompilerOptions().getJvmTarget().set(JvmTarget.JVM_1_8);
            }
        } else if(manager.hasPlugin("java")) {
            JavaPluginExtension extension = target.getExtensions().findByType(JavaPluginExtension.class);
            extension.setTargetCompatibility(version);
            extension.setSourceCompatibility(version);
//            System.out.println("Java plugin found!");

            if(manager.hasPlugin("org.jetbrains.kotlin.jvm")) {
                KotlinJvmExtension kotlinExtension = target.getExtensions().findByType(KotlinJvmExtension.class);
                kotlinExtension.getCompilerOptions().getJvmTarget().set(JvmTarget.JVM_1_8);
            }
        } else {
            throw new RuntimeException("No plugin to configure the java version with was found for project \"" + target.getName() + "\"");
        }


    }

}