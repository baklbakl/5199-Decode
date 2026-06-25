package org.ducktapeandaprayer;

import com.android.build.api.dsl.*;
import org.gradle.api.NamedDomainObjectContainer;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.plugins.ExtensionContainer;
import org.gradle.api.plugins.PluginManager;
import org.gradle.internal.extensibility.ExtensionsStorage;

import java.lang.reflect.Field;
import java.util.Map;

public class AndroidDefaultSetupPlugin implements Plugin<Project> {
    @Override
    public void apply(Project target) {
        PluginManager manager = target.getPluginManager();

        if(manager.hasPlugin("com.android.base")) {
            manager.apply(RepositoriesPlugin.class);
            manager.apply(Java8Plugin.class);

            Class<? extends CommonExtension> extensionClass = ReflectionHelper.getCommonExtensionClass(target);
            CommonExtension extension = target.getExtensions().findByType(extensionClass);

            int compileSDK = PropertiesHelper.getIntProperty(target, "androidsdk.compileVersion");
            extension.setCompileSdk(compileSDK);
            extension.getBuildFeatures().setBuildConfig(true);
            
            Lint lint = extension.getLint();
            lint.setTargetSdk(compileSDK);
            lint.setAbortOnError(false);

            DefaultConfig defaultConfig = extension.getDefaultConfig();
            defaultConfig.setMinSdk(PropertiesHelper.getIntProperty(target, "androidsdk.minVersion"));

            AbiSplit splits = extension.getSplits().getAbi();
            splits.setEnable(true);
            splits.setUniversalApk(false);
            splits.reset();
            splits.include("arm64-v8a", "armeabi-v7a");

            NamedDomainObjectContainer<? extends BuildType> buildTypes = extension.getBuildTypes();
            BuildType debugBuild = buildTypes.getByName("debug");
            debugBuild.setJniDebuggable(true);
            ReflectionHelper.invokeTried(ReflectionHelper.getMethodTried(debugBuild.getClass(), "setDefault", boolean.class), debugBuild, true);
            ReflectionHelper.invokeTried(ReflectionHelper.getMethodTried(debugBuild.getClass(), "setDebuggable", boolean.class), debugBuild, true);

            buildTypes.getByName("debug").setMinifyEnabled(false);

            extension.setEnableKotlin(false);
        } else {
            throw new RuntimeException("AndroidDefaultSetupPlugin applied, but com.android.base wasn't found for project \"" + target.getName() + "\"");
        }
    }
}
