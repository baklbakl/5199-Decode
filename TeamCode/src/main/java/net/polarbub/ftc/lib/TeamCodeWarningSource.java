package net.polarbub.ftc.lib;

import com.qualcomm.robotcore.util.GlobalWarningSource;

public class TeamCodeWarningSource extends RequireSingleInstance implements GlobalWarningSource {
    private static TeamCodeWarningSource instance = null;

    public static TeamCodeWarningSource getInstance() {
        return instance;
    }

    public TeamCodeWarningSource() {
        if(instance != null) {
            throw new RuntimeException("More than one instance of " + this.getClass().getCanonicalName() + " should not be created");
        }

        instance = this;
    }

    private int suppressionCount = 0;
    private String warning  = "";

    @Override
    public String getGlobalWarning() {
        if (suppressionCount > 0) {
            return "";
        }

        return warning;
    }

    @Override
    public boolean shouldTriggerWarningSound() {
        return false;
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {
        if (suppress)
            suppressionCount++;
        else
            suppressionCount--;
    }

    @Override
    public void setGlobalWarning(String warning) {
        this.warning = warning;
    }

    @Override
    public void clearGlobalWarning() {
        this.warning = "";
    }
}
