package net.polarbub.ftc.lib.optiondriver;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

/**
 * A ControllerOption that will cycle through all the elements of an enum. This uses the toString() method of the enum to get the string value of each element.
 */
public class EnumOption<T extends Enum<T>> extends GenericOption {
    private final List<T> enumElements = new ArrayList<>();
    private final Field outVar;
    private final Object outObject;

    private int state = 0;

    /**
     * @param description The description of the option
     * @param button The button of gamepad to use
     * @param gamepad The gamepad instance to read from
     * @param outVar The Field to output to
     * @param outObject The Object that contains that field
     * @param enumClass The enum that you want to toggle through. Use [Class Name].class for this
     */
    public EnumOption(String description, String button, Object gamepad, Field outVar, Object outObject, Class<T> enumClass) {
        super(description, button, gamepad);
        enumElements.addAll(EnumSet.allOf(enumClass));
        try {
            T t = (T) outVar.get(outObject);
            if(t == null) {
                outVar.set(outObject, enumElements.get(0));
            } else {
                state = enumElements.indexOf(t);
            }
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
        this.outVar = outVar;
        this.outObject = outObject;
    }


    String getCurrentString() {
        return enumElements.get(state).toString();
    }

    void buttonPressed() {
        state++;
        state %= enumElements.size();

        try {
            outVar.set(outObject, enumElements.get(state));
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }
}
