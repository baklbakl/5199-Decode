package net.polarbub.ftc.lib.variablewatcher;

//import net.polarbub.ftc.lib.lang3.ToBooleanBiFunction;
import org.apache.commons.lang3.function.ToBooleanBiFunction;

import java.lang.reflect.Field;
import java.util.function.BiConsumer;

public class DoubleVariableWatcher<T> extends ImplVariableWatcher<T> {
    public DoubleVariableWatcher(Object fieldContainer, Field toWatch, Field secondFieldToWatch, ToBooleanBiFunction<T, T> equivalenceChecker, BiConsumer<T, T> function) {
        super(fieldContainer, new Field[]{toWatch, secondFieldToWatch}, equivalenceChecker, (T[] ts) -> {
            function.accept(ts[0], ts[1]);
        });
    }

    public DoubleVariableWatcher(Object fieldContainer, Field toWatch, Field secondFieldToWatch, BiConsumer<T, T> function) {
        this(fieldContainer, toWatch, secondFieldToWatch, null, function);
    }
}
