package net.polarbub.ftc.lib.optiondriver;

import java.lang.reflect.Field;

/**
 * A ControllerOption that will change a boolean between true and false
 */
public class BooleanOption extends GenericOption {
    private boolean toggled = false;
    private String on = "✅";
    private String off = "❌";
    private Field outBoolean = null;
    private Object outObject = null;

    private void commonConstructor(String description, String button, Object gamepad, Field outVar, Object outObject) {
        this.outBoolean = outVar;
        this.outObject = outObject;
        try {
            toggled = (boolean) outVar.get(outObject);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * The string conversion is done by using toString() on the values. See the other constructor when you need to specify a string
     *
     * @param description The description of the option
     * @param button      The button of gamepad to use
     * @param gamepad     The gamepad instance to read from
     * @param outBoolean  The Boolean Field to output to
     * @param outObject   The Object that contains that field
     */
    public BooleanOption(String description, String button, Object gamepad, Field outBoolean, Object outObject) {
        super(description, button, gamepad);
        commonConstructor(description, button, gamepad, outBoolean, outObject);
    }

    /**
     * @param button       The button of gamepad to use
     * @param description  The description of the option
     * @param gamepad      The gamepad instance to read from
     * @param outBoolean   The Boolean Field to output to
     * @param outObject    The Object that contains that field
     * @param onString The string conversion of onValue
     * @param defaultString The string conversion of defaultValue
     */
    public BooleanOption(String description, String button, Object gamepad, Field outBoolean, Object outObject, String onString, String defaultString) {
        super(description, button, gamepad);
        commonConstructor(description, button, gamepad, outBoolean, outObject);
        this.on = onString;
        this.off = defaultString;
    }

    public String getCurrentString() {
        return toggled ? on : off;
    }

    public void buttonPressed() {
        toggled = !toggled;
        try {
            outBoolean.set(outObject, toggled);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }
}
