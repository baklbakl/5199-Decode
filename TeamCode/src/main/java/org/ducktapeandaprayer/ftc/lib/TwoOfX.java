package org.ducktapeandaprayer.ftc.lib;

public class TwoOfX {
    public static boolean twoOfThree(boolean one, boolean two, boolean three) {
        return one ? (two || three) : (two && three);
    }

    public static boolean twoOfFour(boolean one, boolean two, boolean three, boolean four) {
        return one ? two || three || four : twoOfThree(two, three, four);
    }
}
