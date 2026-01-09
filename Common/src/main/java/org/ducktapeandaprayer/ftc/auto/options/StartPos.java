package org.ducktapeandaprayer.ftc.auto.options;

public enum StartPos {
    GOAL("Touching goal"),
    CLASSIFIER("Touching classifier ramp"),
    AUDIENCE_IN("In audience launch zone"),
    AUDIENCE_OUT("Just overlapping audience launch zone"),

    ;

    StartPos(String conversion) {
        this.conversion = conversion;
    }

    private final String conversion;

    @Override
    public String toString() {
        return conversion;
    }
}

