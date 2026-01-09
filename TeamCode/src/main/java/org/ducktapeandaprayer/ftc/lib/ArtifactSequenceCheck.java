package org.ducktapeandaprayer.ftc.lib;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class ArtifactSequenceCheck {

    private final Artifact[] expectedOrder = new Artifact[3];
    private int expectedIndex = 0;

    public ArtifactSequenceCheck(Artifact expected1, Artifact expected2, Artifact expected3) {
        expectedOrder[0] = expected1;
        expectedOrder[1] = expected2;
        expectedOrder[2] = expected3;
    }

    public void resetExpectedProgress() {
        expectedIndex = 0;
        ArtifactList.clearList();
    }


    public boolean isExpectedComplete() {
        return expectedIndex >= expectedOrder.length;
    }


    public boolean onIntake(ColorSensor sensor) {
        Artifact seen = classify(sensor);
        if (seen == null) {
            // TODO  make it do sum - this is lowk a later prob
            return true;
        }

        ArtifactList.addIntake(seen);
        if (isExpectedComplete()) return true;

        Artifact expectedNow = expectedOrder[expectedIndex];
        boolean correct = (seen == expectedNow);
        if (correct) {
            expectedIndex++;
        }
        return correct;
    }

    public void onRelease() {
        ArtifactList.releaseOldest();
    }


    private Artifact classify(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        // If too dim/desaturated, skip
        if (val < 0.10f || sat < 0.12f) return null;

        // TODO add this to MainBotConfig so i can edit w/ FTC dask
        if (hue >= 260 && hue <= 300) return Artifact.PURPLE;
        if (hue >= 90  && hue <= 160) return Artifact.GREEN;

        return null;
    }

    private static final class ArtifactList {

        static void addIntake(Artifact a) {
            if (Artifact.artifactOne == null) {
                Artifact.artifactOne = a;
            } else if (Artifact.artifactTwo == null) {
                Artifact.artifactTwo = a;
            } else if (Artifact.artifactThree == null) {
                Artifact.artifactThree = a;
            } else {
                releaseOldest();
                Artifact.artifactThree = a;
            }
        }


        static void releaseOldest() {
            Artifact.artifactOne = Artifact.artifactTwo;
            Artifact.artifactTwo = Artifact.artifactThree;
            Artifact.artifactThree = null;
        }

        static void clearList() {
            Artifact.artifactOne = null;
            Artifact.artifactTwo = null;
            Artifact.artifactThree = null;
        }

    }
}