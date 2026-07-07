package org.ducktapeandaprayer.ftc.lib.variablewatcher;

import org.apache.commons.lang3.function.ToBooleanBiFunction;
import org.apache.commons.lang3.function.TriConsumer;
import org.ducktapeandaprayer.ftc.lib.QuadConsumer;

import java.lang.reflect.Field;

public class QuadVariableWatcher<T> extends ImplVariableWatcher<T> {
    public QuadVariableWatcher(Object fieldContainer, Field toWatch, Field secondFieldToWatch, Field thirdFieldToWatch, Field fourthFieldToWatch, ToBooleanBiFunction<T, T> equivalenceChecker, QuadConsumer<T, T, T, T> function) {
        super(fieldContainer, new Field[]{toWatch, secondFieldToWatch, thirdFieldToWatch, fourthFieldToWatch}, equivalenceChecker, (T[] ts) -> {
            function.accept(ts[0], ts[1], ts[2], ts[3]);
        });
    }

    public QuadVariableWatcher(Object fieldContainer, Field toWatch, Field secondFieldToWatch, Field thirdFieldToWatch, Field fourthFieldToWatch, QuadConsumer<T, T, T, T> function) {
        this(fieldContainer, toWatch, secondFieldToWatch, thirdFieldToWatch, fourthFieldToWatch, null, function);
    }
}
