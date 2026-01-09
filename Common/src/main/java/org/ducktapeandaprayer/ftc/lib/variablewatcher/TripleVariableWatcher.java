package org.ducktapeandaprayer.ftc.lib.variablewatcher;

import org.apache.commons.lang3.function.ToBooleanBiFunction;
import org.apache.commons.lang3.function.TriConsumer;

import java.lang.reflect.Field;

public class TripleVariableWatcher<T> extends ImplVariableWatcher<T> {
    public TripleVariableWatcher(Object fieldContainer, Field toWatch, Field secondFieldToWatch, Field thirdFieldToWatch, ToBooleanBiFunction<T, T> equivalenceChecker, TriConsumer<T, T, T> function) {
        super(fieldContainer, new Field[]{toWatch, secondFieldToWatch, thirdFieldToWatch}, equivalenceChecker, (T[] ts) -> {
            function.accept(ts[0], ts[1], ts[2]);
        });
    }

    public TripleVariableWatcher(Object fieldContainer, Field toWatch, Field secondFieldToWatch, Field thirdFieldToWatch, TriConsumer<T, T, T> function) {
        this(fieldContainer, toWatch, secondFieldToWatch, thirdFieldToWatch, null, function);
    }
}
