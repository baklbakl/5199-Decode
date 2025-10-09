package net.polarbub;

import com.android.build.api.dsl.*;
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
            Class<? extends CommonExtension> extensionClass = ReflectionHelper.getCommonExtensionClass(target);

            //This big reflection mess is due to the fact that the ApplicationExtension and LibraryExtension types have
            // different types for their flavor related methods (they don't inherit from each other), but they have the
            // same method names that do the same things, so reflection can be used to do both with a smaller amount of code.
            CommonExtension commExtension = target.getExtensions().findByType(extensionClass);

            String dimensionName = "libSource";
            ((List<String>) ReflectionHelper.invokeTried(ReflectionHelper.getMethodTried(extensionClass, "getFlavorDimensions"), commExtension)).add(dimensionName);

            Object namedObjectContainer = ReflectionHelper.invokeTried(ReflectionHelper.getMethodTried(extensionClass, "getProductFlavors"), commExtension);
            Method createMethod = ReflectionHelper.getMethodTried(namedObjectContainer.getClass(), "create",  String.class, Action.class);

            ReflectionHelper.invokeTried(createMethod, namedObjectContainer, "aarLibs", (Action<?>) ((Object flavor) -> {
                ReflectionHelper.invokeTried(ReflectionHelper.getMethodTried(flavor.getClass(), "setDefault", boolean.class), flavor, true);
                ReflectionHelper.invokeTried(ReflectionHelper.getMethodTried(flavor.getClass(), "setDimension", String.class), flavor, dimensionName);
            }));

            ReflectionHelper.invokeTried(createMethod, namedObjectContainer, "extractedLibs", (Action<?>) ((Object flavor) -> {
                ReflectionHelper.invokeTried(ReflectionHelper.getMethodTried(flavor.getClass(), "setDimension", String.class), flavor, dimensionName);
            }));
        } else {
            throw new RuntimeException("FTCLibFlavorPlugin applied, but com.android.base wasn't found for project \"" + target.getName() + "\"");
        }
    }
}
