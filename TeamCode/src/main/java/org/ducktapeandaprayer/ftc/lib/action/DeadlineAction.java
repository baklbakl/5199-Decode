package org.ducktapeandaprayer.ftc.lib.action;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class DeadlineAction implements Action {
    private final Action deadline;
    private final Action other;
    private boolean otherRunAgain = true;

    public DeadlineAction(Action deadline, Action other) {
        this.deadline = deadline;
        this.other = other;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(!deadline.run(telemetryPacket)) {
            return false;
        }

        if(otherRunAgain) {
            otherRunAgain = other.run(telemetryPacket);
        }

        return true;
    }
}
