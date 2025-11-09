package net.polarbub.ftc.lib;

import com.qualcomm.robotcore.util.RobotLog;

import java.lang.ref.WeakReference;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class SingletonHelper {
    public static Map<Class<?>, WeakReference<?>> instances = new HashMap<>();

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
}