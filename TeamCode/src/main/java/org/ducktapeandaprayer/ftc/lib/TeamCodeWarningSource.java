package org.ducktapeandaprayer.ftc.lib;

import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.BuildConfig;

import java.util.ArrayList;
import java.util.List;

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
//    private List<String> warnings = new ArrayList<>();
    private String warning  = "";

    @Override
    public String getGlobalWarning() {
        if (suppressionCount > 0) {
            return "";
        }

        List<String> warnings = new ArrayList<>();
        warnings.add(warning);

        if(BuildConfig.FLAVOR.equals("extractedLibs")) {
            warnings.add("NOT LEGAL FOR COMP!!!! Using extracted FTC libraries. To use legal AAR libs switch build flavor to \"aarLibs\"");
        } else if (!BuildConfig.FLAVOR.equals("aarLibs")) {
            warnings.add("Unknown build flavor. Probably not using the official AAR libs and is NOT legal for comp");
        }

        return RobotLog.combineGlobalWarnings(warnings);
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
    public void setGlobalWarning(String warning) {}

    public void setWarning(String warning) {
        this.warning = warning;
    }

    @Override
    public void clearGlobalWarning() {}

    public void clearWarning() {
        this.warning = "";
    }
}
