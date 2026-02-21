package org.ducktapeandaprayer.ftc.vision;

import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.opencv.core.Mat;
import org.openftc.easyopencv.*;

import java.util.concurrent.TimeUnit;

public abstract class PipelineStarter extends OpenCvPipeline {
    public static int gain = 180; // Min 0, max 255
    public static long exposure = 8; // Max 204, min 0.
    public static long framesBeforeSet = 8; // Max 204, min 0.

    public final OpenCvWebcam camera;

    public PipelineStarter(MainBotHardware hardware, int width, int height) {
        int cameraMonitorViewId = hardware.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardware.hardwareMap.appContext.getPackageName());
        if(hardware.webcamName == null) {
            throw new RuntimeException("Pipeline Starter / Vision cannot be started without a camera");
        }
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardware.webcamName, cameraMonitorViewId);
        camera.setPipeline(this);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.YUY2);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public PipelineStarter(MainBotHardware hardware) {
        this(hardware, 800, 448);
    }

    public void updateCameraSettings() {
        if(camera.getExposureControl().getMode() != ExposureControl.Mode.Manual) {
            camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
        }

        if(camera.getGainControl().getGain() != gain) {
            camera.getGainControl().setGain(gain);
        }

        if(camera.getExposureControl().getExposure(TimeUnit.MILLISECONDS) != exposure) {
            camera.getExposureControl().setExposure(exposure, TimeUnit.MILLISECONDS);
        }
    }

    public String getTimingTelemetry() {
        StringBuilder sb = new StringBuilder();
        sb.append("FPS: ").append(camera.getFps()).append('\n');
        sb.append("Ms per frame processing: ").append(camera.getPipelineTimeMs()).append('\n');
        sb.append("Ms per frame overhead: ").append(camera.getOverheadTimeMs()).append('\n');
        sb.append("Ms per frame total: ").append(camera.getTotalFrameTimeMs());
        return sb.toString();
    }

    private int frameCount = 0;

    //TODO: Document why we apply the settings like this ...
    @Override
    public Mat processFrame(Mat mat) {
        if(frameCount < framesBeforeSet) {
            frameCount++;
        } else if(frameCount == framesBeforeSet) {
            frameCount++;
            updateCameraSettings();
        }
        return childProcessFrame(mat);
    }

    public abstract Mat childProcessFrame(Mat mat);
}
