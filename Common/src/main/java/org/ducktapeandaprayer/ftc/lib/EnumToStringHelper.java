package org.ducktapeandaprayer.ftc.lib;

import org.apache.commons.lang3.StringUtils;

public class EnumToStringHelper {
    //Converts strings in the formate of "ONE_TWO_THREE" to "One two three"
    public static String toStringHelper(String stock) {
        return StringUtils.capitalize(stock.replace("_", " ").toLowerCase());
    }
}