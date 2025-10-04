package net.polarbub.ftc.auto.options;

public enum Delay {
    NONE,
    FIVE,
    TEN
    ;
    @Override
    public String toString() {
        if(this == NONE) {
            return "0";
        } else if(this == FIVE) {
            return "5";
        } else if (this == TEN) {
            return "10";
        }
        throw new RuntimeException("Delay toString() incomplete");
    }
}
