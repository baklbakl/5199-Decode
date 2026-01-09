package org.ducktapeandaprayer.ftc.lib;

import java.lang.ref.WeakReference;
import java.util.HashMap;
import java.util.Map;

//Make another class extend this class and that class will be guaranteed to only ever have one instance at a time
public class RequireSingleInstance {
    public static Map<Class<? extends RequireSingleInstance>, WeakReference<RequireSingleInstance>> instances = new HashMap<>();

    public RequireSingleInstance() {
        //Must GC to make sure that everything is no longer referenced is cleared out otherwise there will be false positives
        System.gc();
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        System.gc();
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        WeakReference<RequireSingleInstance> instance = instances.get(this.getClass());
        if(instance == null || instance.get() == null) {
            instances.put(this.getClass(), new WeakReference<>(this));
        } else {
            throw new RuntimeException("Only one instance of " + this.getClass().getCanonicalName() + " should exist at once!");
        }
    }
}