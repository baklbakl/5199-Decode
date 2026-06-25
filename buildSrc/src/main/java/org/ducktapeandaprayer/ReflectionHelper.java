package org.ducktapeandaprayer;

import com.android.build.api.dsl.ApplicationExtension;
import com.android.build.api.dsl.LibraryExtension;
import com.android.build.api.dsl.CommonExtension;
import org.gradle.api.Project;
import org.gradle.api.plugins.PluginManager;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

public class ReflectionHelper {


    public static Class<? extends CommonExtension> getCommonExtensionClass (Project target) {
        Class<? extends CommonExtension> extensionClass;

        PluginManager manager = target.getPluginManager();

        if(manager.hasPlugin("com.android.application")) {
            extensionClass = ApplicationExtension.class;

        } else if (manager.hasPlugin("com.android.library")) {
            extensionClass = LibraryExtension.class;

        } else {
            throw new RuntimeException("Unknown android plugin for project \"" + target.getName() + "\"");
        }

        return extensionClass;
    }

    static Method getMethodTried(Class<?> clazz, String name, Class<?>... paramTypes) {
        try {
            return clazz.getMethod(name, paramTypes);
        } catch (NoSuchMethodException e) {
            throw new RuntimeException(e);
        }
    }

    static Object invokeTried(Method method, Object obj, Object... args) {
        try {
            return method.invoke(obj, args);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }
}
