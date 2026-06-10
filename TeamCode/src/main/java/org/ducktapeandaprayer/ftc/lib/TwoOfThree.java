package org.ducktapeandaprayer.ftc.lib;

public class TwoOfThree {
    public static boolean twoOfThree(boolean one, boolean two, boolean three) {
        return one ? (two || three) : (two && three);
    }
}
