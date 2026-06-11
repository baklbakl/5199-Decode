package org.ducktapeandaprayer.ftc.auto.options;

public enum GatePush {
    AFTER_ONE("After First Spike"),
    AFTER_TWO("After Second Spike"),
    AFTER_BOTH("After both first and second spike"),
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
