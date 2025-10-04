package net.polarbub.ftc.lib.optiondriver;

import java.util.ArrayList;

public class OptionDriver extends ArrayList<GenericOption> {

    @Override
    public String toString() {
        StringBuilder stringBuilder = new StringBuilder();
        for(GenericOption option : this) {
            stringBuilder.append(option.getDescription());
            stringBuilder.append(" - ");
            stringBuilder.append(option.getCurrentString());
            stringBuilder.append(" (");
            stringBuilder.append(option.getButtonName());
            stringBuilder.append(")\n");
        }

        return stringBuilder.toString();
    }

    private String lastOutputted = "";

    public String tick() {
        for(GenericOption option : this) {
            option.tick();
        }

        String toString = toString();
        if(!toString.equals(lastOutputted)) {
            lastOutputted = toString;
            return lastOutputted;
        } else return null;
    }
}