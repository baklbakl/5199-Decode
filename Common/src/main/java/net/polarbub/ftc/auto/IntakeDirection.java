package net.polarbub.ftc.auto;

import net.polarbub.ftc.lib.EnumToStringHelper;
import org.apache.commons.lang3.StringUtils;

public enum IntakeDirection {
    IN,
    OUT,
    STOP;

    @Override
    public String toString() {
        return EnumToStringHelper.toStringHelper(super.toString());
    }
}
