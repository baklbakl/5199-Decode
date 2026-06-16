package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.action.TimeoutAction;

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

    public Action makeFollowerAction(List<Pose2d> poses) {
        List<Pose2d> mirroredPoses = new ArrayList<>();
        for(Pose2d pose : poses) {
            mirroredPoses.add(invertIfBlue.apply(pose));
        }
        return new TimeoutAction(new FollowerAction(follower, dataBlockSupplier, mirroredPoses), 3.0);
    }

    public Action makeFollowerAction(Pose2d pose) {
        return makeFollowerAction(List.of(pose));
    }
}
