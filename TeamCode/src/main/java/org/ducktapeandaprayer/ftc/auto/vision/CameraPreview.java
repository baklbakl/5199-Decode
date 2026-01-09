package org.ducktapeandaprayer.ftc.auto.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.opencv.core.Mat;

@Autonomous(name = "Camera Preview")
public class CameraPreview extends LinearOpMode {
    @Override
    public void runOpMode() {
        CameraPreviewPipline vision = new CameraPreviewPipline(MainBotHardware.getOrCreate(hardwareMap));
        while (opModeInInit() && !isStopRequested()) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException ignored) {}
        }
    }
}

class CameraPreviewPipline extends PipelineStarter {
    public CameraPreviewPipline(MainBotHardware hardware) {
        super(hardware);
    }

    @Override
    public Mat processFrame(Mat input) {
//        return input.submat(360,720,0,1280);
        return input;
    }
}