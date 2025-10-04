package net.polarbub.ftc.lib.optiondriver;
import java.lang.reflect.Field;

abstract class GenericOption {
    abstract String getCurrentString();
    abstract void buttonPressed();

    public  String getDescription() {
        return description;
    }

    public String getButtonName() {
        return buttonName;
    }

    private String buttonName;
    private String description;
    private Field button = null;
    private Object gamepad = null;
    private boolean lock = false;


    GenericOption(String description, String button, Object gamepad) {
        try {
            this.button = gamepad.getClass().getDeclaredField(button);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
        this.buttonName =  this.button.getName();
        this.gamepad = gamepad;
        this.description = description;
    }

    public void tick() {
        try {
            boolean buttonPressed = button.getBoolean(gamepad);
            if(buttonPressed && !lock) {
                lock = true;
                buttonPressed();
            } else if(!buttonPressed && lock) {
                lock = false;
            }
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }
}
