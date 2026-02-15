package org.ducktapeandaprayer.ftc.auto.options;

public enum Delay {
    NONE("0"),
    FIVE("5"),
    TEN("10")
    ;

    private final String string;

    @Override
    public String toString() {
        return string;
    }

    Delay(String string) {
        this.string = string;
    }
}
