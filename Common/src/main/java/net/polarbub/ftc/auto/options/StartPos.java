package net.polarbub.ftc.auto.options;

public enum StartPos {
    TAG,
    NEXT_TAG,
    LAUNCH;

    @Override
    public String toString() {
        if(this == TAG) {
            return "in front of April Tag";
        } else if (this == NEXT_TAG) {
            return  "right wheel next to line (April Tag)";
        } else if (this == LAUNCH) {
            return "left wheel on end (launch zone line)";
        }
        throw new RuntimeException("StartPOS toString() incomplete");
    }
}

