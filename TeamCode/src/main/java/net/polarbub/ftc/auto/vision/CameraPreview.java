package net.polarbub.ftc.auto.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import static net.polarbub.ftc.auto.PipelineStarter.startPipeline;

@Autonomous(name = "Camera Preview")
public class CameraPreview extends LinearOpMode {
    @Override
    public void runOpMode() {
        CameraPreviewPipline vision = new CameraPreviewPipline(hardwareMap);
        while (opModeInInit() && !isStopRequested()) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException ignored) {}
        }
    }
}

class CameraPreviewPipline extends OpenCvPipeline {
    public CameraPreviewPipline(HardwareMap hardwareMap) {
        startPipeline(this, hardwareMap);
    }

    @Override
    public Mat processFrame(Mat input) {
//        return input.submat(360,720,0,1280);
        return input;
    }
}