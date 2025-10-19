package net.polarbub.ftc.lib;

public enum Artifact {
    PURPLE,
    GREEN;

    @Override
    public String toString() {
        if (this == PURPLE) {
            return "🟪 (purple)";
        } else if (this == GREEN) {
            return "🟩 (blue)";
        }
        throw new RuntimeException("Alliance toString() incomplete");
    }

    public static Artifact artifactOne = null;
    public static Artifact artifactTwo = null;
    public static Artifact artifactThree = null;
}
