package org.ducktapeandaprayer.ftc.auto.roadrunner;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.jetbrains.annotations.NotNull;

public class OctoQuadEncoder implements Encoder {
    private final OctoQuad octoQuad;
    private final int index;
    private final int samplingInterval;
    //Don't change the sampling interval mid opmode please
    public OctoQuadEncoder(OctoQuad octoQuad, int index) {
        this.octoQuad = octoQuad;
        this.index = index;
        this.samplingInterval = octoQuad.getSingleVelocitySampleInterval(index);
    }

    @NonNull
    @Override
    public DcMotorSimple.Direction getDirection() {
        return DcMotorSimple.Direction.FORWARD;
    }

    @Override
    public void setDirection(@NonNull DcMotorSimple.Direction direction) {}

    @Override
    public @NotNull PositionVelocityPair getPositionAndVelocity() {
        int pos = octoQuad.readSinglePosition_Caching(index);
        int vel = octoQuad.readSingleVelocity_Caching(index) * 1000 / samplingInterval;
        return new PositionVelocityPair(pos, vel, pos, vel);
    }
}
