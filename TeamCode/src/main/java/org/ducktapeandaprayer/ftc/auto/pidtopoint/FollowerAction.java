package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import org.ducktapeandaprayer.ftc.lib.SetupAction;

import java.util.List;
import java.util.function.Supplier;

public class FollowerAction extends SetupAction {
    private final Follower follower;
    private final List<Pose2d> poses;
    private final Supplier<OctoQuad.LocalizerDataBlock> dataBlockSupplier;

    public FollowerAction(Follower follower, Supplier<OctoQuad.LocalizerDataBlock> dataBlockSupplier, List<Pose2d> poses) {
        this.follower = follower;
        this.poses = poses;
        this.dataBlockSupplier = dataBlockSupplier;
    }

    @Override
    public void preview(@NonNull Canvas fieldOverlay) {
        //ADD: Dash preview for FollowerAction
        super.preview(fieldOverlay);
    }

    @Override
    public void setup(@NonNull TelemetryPacket telemetryPacket) {
        follower.followTrajectory(poses, false);
    }

    @Override
    public boolean subRun(@NonNull TelemetryPacket telemetryPacket) {
        OctoQuad.LocalizerDataBlock dataBlock = dataBlockSupplier.get();
        if(dataBlock == null) {
            return false;
        }

        follower.tick(dataBlock);
        return !follower.isCanceled();
    }
}
