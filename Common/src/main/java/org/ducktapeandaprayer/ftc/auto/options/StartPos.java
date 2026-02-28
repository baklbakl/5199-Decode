package org.ducktapeandaprayer.ftc.auto.options;

public enum StartPos {
    GOAL("Touching goal"),
    CLASSIFIER("Touching classifier ramp"),
    AUDIENCE("Audience launch zone"),
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

