package org.ducktapeandaprayer.ftc.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Camera Preview")
public class CameraPreview extends LinearOpMode {
    @Override
    public void runOpMode() {
        CameraPreviewPipline vision = new CameraPreviewPipline(MainBotHardware.getOrCreate(hardwareMap));
        ArrayList<VariableWatcher> watchers = new ArrayList<>();
        try {
            watchers.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("square"), (Boolean square) -> {
                if(square) {
                    vision.updateCameraSettings();
                }
            }));
            watchers.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("circle"), (Boolean circle) -> {
                if(circle) {
                    vision.showCenterLine = !vision.showCenterLine;
                }
            }));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        telemetry.addLine("Square to update camera settings");
        telemetry.addLine("Circle to turn on center line");
        telemetry.addLine("These only work when the camera stream is closed");
        telemetry.update();

        while (opModeInInit() && !isStopRequested()) {
            for(VariableWatcher watcher : watchers) {
                watcher.tick();
            }
            try {
                Thread.sleep(1);
            } catch (InterruptedException ignored) {}
        }
    }
}

class CameraPreviewPipline extends PipelineStarter {
    public volatile boolean showCenterLine = false;

    public CameraPreviewPipline(MainBotHardware hardware) {
        super(hardware);
    }

    @Override
    public Mat childProcessFrame(Mat input) {
//        return input.submat(360,720,0,1280);
        if(showCenterLine) {
            Imgproc.line(input, new Point(input.width() / 2, 0), new Point(input.width() / 2, input.height()), new Scalar(0, 255, 0), 1);
        }
        return input;
    }
}