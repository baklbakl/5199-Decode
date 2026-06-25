package org.ducktapeandaprayer;

import com.android.build.api.dsl.*;

import org.gradle.api.NamedDomainObjectContainer;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.Action;
import org.gradle.api.plugins.PluginManager;

import java.lang.reflect.Method;
import java.util.List;


public class FTCLibFlavorPlugin implements Plugin<Project> {
    @Override
    public void apply(Project target) {
        PluginManager manager = target.getPluginManager();
        if(manager.hasPlugin("com.android.base")) {
            CommonExtension commExtension = target.getExtensions().findByType(CommonExtension.class);

            String dimensionName = "libSource";
            commExtension.getFlavorDimensions().add(dimensionName);
            NamedDomainObjectContainer<? extends ProductFlavor> productFlavors = commExtension.getProductFlavors();
            productFlavors.create("aarLibs", ((ProductFlavor flavor) -> {
                //For some reason setDefault isn't a method of ProductFlavor, only
                // LibraryProductFlavor and ApplicationProductFlavor, so we use reflection to avoid code duplication
                ReflectionHelper.invokeTried(ReflectionHelper.getMethodTried(flavor.getClass(), "setDefault", boolean.class), flavor, true);
                flavor.setDimension(dimensionName);
            }));

            productFlavors.create("extractedLibs", (ProductFlavor flavor) -> {
                flavor.setDimension(dimensionName);
            });
        } else {
            throw new RuntimeException("FTCLibFlavorPlugin applied, but com.android.base wasn't found for project \"" + target.getName() + "\"");
        }
    }
}
