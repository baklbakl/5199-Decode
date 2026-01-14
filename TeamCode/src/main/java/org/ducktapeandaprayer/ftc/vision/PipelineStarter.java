package org.ducktapeandaprayer.ftc.vision;

import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.openftc.easyopencv.*;

public abstract class PipelineStarter extends OpenCvPipeline {
    public final OpenCvWebcam camera;

    public PipelineStarter(MainBotHardware hardware, int width, int height) {
        int cameraMonitorViewId = hardware.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardware.hardwareMap.appContext.getPackageName());
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

    public String getTimingTelemetry() {
        StringBuilder sb = new StringBuilder();
        sb.append("FPS: ").append(camera.getFps()).append('\n');
        sb.append("Ms per frame processing: ").append(camera.getPipelineTimeMs()).append('\n');
        sb.append("Ms per frame overhead: ").append(camera.getOverheadTimeMs()).append('\n');
        sb.append("Ms per frame total: ").append(camera.getTotalFrameTimeMs());
        return sb.toString();
    }
}
