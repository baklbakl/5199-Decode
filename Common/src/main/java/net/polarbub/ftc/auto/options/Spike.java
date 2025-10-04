package net.polarbub.ftc.auto.options;

public enum Spike {
    DONT,
    ONE,
    TWO,
    THREE;

    @Override
    public String toString() {
        if(this == DONT) {
            return "don't";
        } else if(this == ONE) {
            return "one";
        } else if(this == TWO) {
            return "two";
        } else if(this == THREE) {
            return "three";
        }

        throw new RuntimeException("Spike toString() incomplete");
    }

    public int toInt() {
        switch (this) {
            case DONT:
                return 0;
            case ONE:
                return 1;
            case TWO:
                return 2;
            case THREE:
                return 3;
        }
        throw new RuntimeException("Spike toInt() incomplete");
    }
}
