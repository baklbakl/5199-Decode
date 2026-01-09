package org.ducktapeandaprayer.ftc.lib;

public enum Alliance {
    RED,
    BLUE;

    @Override
    public String toString() {
        if (this == RED) {
            return "🟥 (red)";
        } else if (this == BLUE) {
            return "🟦 (blue)";
        }
        throw new RuntimeException("Alliance toString() incomplete");
    }

    public int toAprilTagID() {
        switch (this) {
            case RED:
                return 24;
            case BLUE:
                return 20;
            default:
                throw new IllegalArgumentException("toAprilTagID incomplete");
        }
    }

    public static Alliance lastAlliance = null;
}
