package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import org.ducktapeandaprayer.ftc.lib.Alliance;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

public class FollowerActionFactory {
    private final Follower follower;
    private final Supplier<OctoQuad.LocalizerDataBlock> dataBlockSupplier;
    private Function<Pose2d, Pose2d> invertIfBlue;

    public FollowerActionFactory(Follower follower, Supplier<OctoQuad.LocalizerDataBlock> dataBlockSupplier, Function<Pose2d, Pose2d> invertIfBlue) {
        this.follower = follower;
        this.dataBlockSupplier = dataBlockSupplier;
        this.invertIfBlue = invertIfBlue;
    }

    public FollowerAction makeFollowerAction(List<Pose2d> poses) {
        List<Pose2d> mirroredPoses = new ArrayList<>();
        for(Pose2d pose : poses) {
            mirroredPoses.add(invertIfBlue.apply(pose));
        }
        return new FollowerAction(follower, dataBlockSupplier, mirroredPoses);
        //return new FollowerAction(follower, dataBlockSupplier, poses);
    }

    public FollowerAction makeFollowerAction(Pose2d pose) {
        return makeFollowerAction(List.of(pose));
    }
}
