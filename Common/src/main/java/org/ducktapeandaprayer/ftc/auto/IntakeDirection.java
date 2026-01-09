package org.ducktapeandaprayer.ftc.auto;

import org.ducktapeandaprayer.ftc.lib.EnumToStringHelper;

public enum IntakeDirection {
    IN,
    OUT,
    STOP;

    @Override
    public String toString() {
        return EnumToStringHelper.toStringHelper(super.toString());
    }
}
