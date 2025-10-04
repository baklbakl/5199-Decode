package net.polarbub.ftc.auto.options;

public enum Score {
    CHAMBER,
    BASKET,
    DONT;

    @Override
    public String toString() {
        if(this == DONT) {
            return "don't";
        } else if(this == CHAMBER) {
            return "on chamber";
        }
        else if(this == BASKET) {
            return "in basket";
        } else {
            throw new RuntimeException("Score toString() incomplete");
        }
    }
}
