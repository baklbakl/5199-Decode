package org.ducktapeandaprayer.ftc.auto.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;

import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;

import java.util.function.Supplier;

public final class OctoQuadLocalizer implements Localizer {
    public final MainBotHardware hardware;
    private Pose2d pose;
    private final Supplier<Boolean> isStopRequested;

    public OctoQuadLocalizer(MainBotHardware hardware, Pose2d initialPose, Supplier<Boolean> isStopRequested) {
        this.hardware = hardware;
        this.isStopRequested = isStopRequested;
        hardware.setOctoQuadPose(initialPose);
        pose = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        hardware.setOctoQuadPose(pose);
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    private PoseVelocity2d velocity = null;
    public PoseVelocity2d getPoseVelocity() {
        return velocity == null ? new PoseVelocity2d(new Vector2d(0,0),0) : velocity;
    }

    @Override
    public PoseVelocity2d update() {
        OctoQuad.LocalizerDataBlock block = hardware.readOctoQuadLocalizerData(isStopRequested);
        pose = MainBotHardware.makeOctoQuadPose(block);
        velocity = new PoseVelocity2d(new Vector2d(block.velX_mmS / 25.4, block.velY_mmS / 25.4), block.velHeading_radS);
        return velocity;
    }
}