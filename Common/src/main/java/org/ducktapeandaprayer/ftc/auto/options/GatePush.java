package org.ducktapeandaprayer.ftc.auto.options;

public enum GatePush {
    AFTER_ONE("After Spike One"),
    AFTER_TWO("After Spike Two"),
    AFTER_BOTH("After both spike one and two"),
    DONT("Don't")
    ;

    GatePush(String conversion) {
        this.conversion = conversion;
    }

    private final String conversion;

    @Override
    public String toString() {
        return conversion;
    }
}
