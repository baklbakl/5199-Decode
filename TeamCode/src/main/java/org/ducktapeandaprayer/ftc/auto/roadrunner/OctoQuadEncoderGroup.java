package org.ducktapeandaprayer.ftc.auto.roadrunner;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.jetbrains.annotations.NotNull;

import java.util.Collections;
import java.util.List;

public class OctoQuadEncoderGroup implements EncoderGroup {
    private final OctoQuad octoQuad;
    private final int xIndex;
    private final int yIndex;

    public OctoQuadEncoderGroup(OctoQuad octoQuad, int xIndex, int yIndex) {
        this.octoQuad = octoQuad;
        this.xIndex = xIndex;
        this.yIndex = yIndex;
    }

    @Override
    public void bulkRead() {
        octoQuad.refreshCache();
    }

    @Override
    public @NotNull List<Encoder> getEncoders() {
        return List.of(new OctoQuadEncoder(octoQuad, xIndex), new OctoQuadEncoder(octoQuad, yIndex));
    }

    @Override
    @NotNull
    public List<Encoder> getUnwrappedEncoders() {
        return getEncoders();
    }
}
