package org.ducktapeandaprayer.ftc.lib.optiondriver;

import java.lang.reflect.Field;
import java.security.InvalidParameterException;

/**
 * A ControllerOption that will move an int through a range of values
 */
public class IntOption extends GenericOption {
    private int state = 0;
    private int bottomNumber = 0;
    private int topNumber = 0;
    private Field outInt = null;
    private Object outObject = null;

    private void commonConstructor(String description, String button, Object gamepad, Field outVar, Object outObject, int bottomNumber, int topNumber) {
        if(bottomNumber >= topNumber) {
            throw new InvalidParameterException("Bottom number must be less than top number");
        }

        this.outInt = outVar;
        this.outObject = outObject;
        this.bottomNumber = bottomNumber;
        this.topNumber = topNumber;
        try {
            state = (int) outVar.get(outObject);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * @param button       The button of gamepad to use
     * @param description  The description of the option
     * @param gamepad      The gamepad instance to read from
     * @param outInt   The Boolean Field to output to
     * @param outObject    The Object that contains that field
     * @param bottomNumber The smallest number in the range of numbers
     * @param topNumber The largest number in the range of numbers
     */
    public IntOption(String description, String button, Object gamepad, Field outInt, Object outObject, int bottomNumber, int topNumber) {
        super(description, button, gamepad);
        commonConstructor(description, button, gamepad, outInt, outObject,bottomNumber, topNumber);
    }

    public String getCurrentString() {
        return String.valueOf(state);
    }

    public void buttonPressed() {
        state++;
        if(state > topNumber) {
            state = bottomNumber;
        }

        try {
            outInt.set(outObject, state);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }
}
