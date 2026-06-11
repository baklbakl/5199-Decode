package org.ducktapeandaprayer.ftc.lib.variablewatcher;

import org.apache.commons.lang3.function.ToBooleanBiFunction;

import java.lang.reflect.Field;
import java.util.function.Consumer;

public class SingleVariableWatcher<T> extends ImplVariableWatcher<T> {
    public SingleVariableWatcher(Object fieldContainer, Field toWatch, ToBooleanBiFunction<T, T> equivalenceChecker, Consumer<T> function) {
        super(fieldContainer, new Field[]{toWatch}, equivalenceChecker, (T[] ts) -> {
            function.accept(ts[0]);
        });
    }

    public SingleVariableWatcher(Object fieldContainer, Field toWatch, Consumer<T> function) {
        this(fieldContainer, toWatch, null, function);
    }
}
