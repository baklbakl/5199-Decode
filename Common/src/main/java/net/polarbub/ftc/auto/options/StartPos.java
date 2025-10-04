package net.polarbub.ftc.auto.options;

public enum StartPos {
    LEFT,
    CENTER,
    RIGHT;

    @Override
    public String toString() {
        if(this == LEFT) {
            return "left of center line";
        } else if (this == CENTER) {
            return  "on center line";
        } else if (this == RIGHT) {
            return "right of center line";
        }
        throw new RuntimeException("StartPOS toString() incomplete");
    }
}

