package net.polarbub.ftc.lib.variablewatcher;

//import net.polarbub.ftc.lib.lang3.ToBooleanBiFunction;
import org.apache.commons.lang3.function.ToBooleanBiFunction;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

class ImplVariableWatcher<T> implements VariableWatcher {
    ImplVariableWatcher(Object fieldContainer, Field[] toWatches, ToBooleanBiFunction<T, T> equivalenceChecker, Consumer<T[]> function) {
        this.toWatches = toWatches;
        oldStates = new ArrayList<>();
        for(int i = 0; i < toWatches.length; i++) {
            oldStates.add(null);
        }
        this.fieldContainer = fieldContainer;
        this.function = function;
        this.equivalenceChecker = equivalenceChecker;
    }

    public void tick() {
        try {
            boolean different = false;

            if(equivalenceChecker == null) {
                for(int i = 0; i < toWatches.length; i++) {
                    T newState = (T) toWatches[i].get(fieldContainer);
                    if(newState != oldStates.get(i)) {
                        different = true;
                        oldStates.set(i, newState);
                    }
                }
            } else {
                for(int i = 0; i < toWatches.length; i++) {
                    T newState = (T) toWatches[i].get(fieldContainer);
                    if(!equivalenceChecker.applyAsBoolean(newState, oldStates.get(i))) {
                        different = true;
                        oldStates.set(i, newState);
                    }
                }
            }

            if(different) function.accept((T[]) oldStates.toArray());
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    private Field[] toWatches;
    private Object fieldContainer;
    private Consumer<T[]> function;
    private List<T> oldStates;
    private ToBooleanBiFunction<T, T> equivalenceChecker;
}
