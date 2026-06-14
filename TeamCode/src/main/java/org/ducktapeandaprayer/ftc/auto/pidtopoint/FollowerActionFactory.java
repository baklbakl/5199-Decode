package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import java.util.List;
import java.util.function.Supplier;

public class FollowerActionFactory {
    private final Follower follower;
    private final Supplier<OctoQuad.LocalizerDataBlock> dataBlockSupplier;

    public FollowerActionFactory(Follower follower, Supplier<OctoQuad.LocalizerDataBlock> dataBlockSupplier) {
        this.follower = follower;
        this.dataBlockSupplier = dataBlockSupplier;
    }

    public FollowerAction makeFollowerAction(List<Pose2d> poses) {
        return new FollowerAction(follower, dataBlockSupplier, poses);
    }

    public FollowerAction makeFollowerAction(Pose2d pose) {
        return new FollowerAction(follower, dataBlockSupplier, List.of(pose));
    }
}
