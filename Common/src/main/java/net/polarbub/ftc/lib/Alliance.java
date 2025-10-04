package net.polarbub.ftc.lib;

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

    public static Alliance lastAlliance = null;
}
