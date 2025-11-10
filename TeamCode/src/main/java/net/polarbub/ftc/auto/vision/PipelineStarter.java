package net.polarbub.ftc.auto.vision;

import net.polarbub.ftc.hardware.MainBotHardware;
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
                camera.startStreaming(width, height, OpenCvCameraRotation.UPSIDE_DOWN, OpenCvWebcam.StreamFormat.YUY2);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public PipelineStarter(MainBotHardware hardware) {
        this(hardware, 1280, 720);
    }
}
