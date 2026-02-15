package org.ducktapeandaprayer.ftc.lib;

public enum Alliance {
    RED("🟥 (red)", 24),
    BLUE("🟦 (blue)", 20);

    private final String string;
    public final int aprilTagID;

    @Override
    public String toString() {
        return string;
    }

    Alliance(String string, int aprilTagID) {
        this.string = string;
        this.aprilTagID = aprilTagID;
    }

    public static Alliance lastAlliance = null;
}
