package net.polarbub.ftc.auto.options;

public enum ParkingPos {
    CORNER,
    EDGE,
    DONT
    ;

    @Override
    public String toString() {
        if(this == CORNER) {
            return "in corner";
        } else if (this == EDGE) {
            return "on edge";
        } else if(this == DONT) {
            return "don't";
        }
        throw new RuntimeException("ParkingPos toString() incomplete");
    }
}
