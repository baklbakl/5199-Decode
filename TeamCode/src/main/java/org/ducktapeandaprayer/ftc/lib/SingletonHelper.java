package org.ducktapeandaprayer.ftc.lib;

import com.qualcomm.robotcore.util.RobotLog;

import java.lang.ref.WeakReference;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class SingletonHelper {
    private static final Map<Class<?>, WeakReference<?>> instances = new HashMap<>();

    public static <T> T getOrCreate(Class<T> clazz, Supplier<T> supplier) {
        WeakReference<T> reference = (WeakReference<T>) instances.get(clazz);
        if(reference == null || reference.get() == null) {
            T instance = supplier.get();
            instances.put(clazz, new WeakReference<>(instance));
            return instance;
        } else {
            RobotLog.logStackTrace(Thread.currentThread(), "WARNING: Class: " + clazz.getCanonicalName() + " got existing reference. This is probably a bug or the GC being slow.");
            return reference.get();
        }
    }

    public static <T> T get(Class<T> clazz) {
        WeakReference<T> reference = (WeakReference<T>) instances.get(clazz);
        if(reference == null) return null;
        return reference.get();
    }

    public static <T> void clear(Class<T> clazz) {
        instances.put(clazz, new WeakReference<>(null));
    }
}