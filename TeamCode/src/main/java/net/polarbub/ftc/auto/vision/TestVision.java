package net.polarbub.ftc.auto.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import net.polarbub.ftc.lib.FastIterativeOpMode;

@Disabled
@Autonomous(name = "Test Vision")
public class TestVision extends FastIterativeOpMode {

    @Override
    protected void stop_internal() {

    }

    @Override
    protected void start_internal() {

    }

    @Override
    protected void loop_internal() {
//        List<FindRedSpecimen.PipelineData> detectedObjects = visionPipeline.getDetectedObjects();
//        List<FindRedSpecimen.PipelineData> previousDetectedObjects = new ArrayList<>();
//
//        if (!detectedObjects.equals(previousDetectedObjects)) {
//            for (int i = 0; i < Math.min(detectedObjects.size(), 3); i++) {
//                FindRedSpecimen.PipelineData data = detectedObjects.get(i);
//                telemetry.addLine(String.format("Object %d:", i + 1));
//                telemetry.addData("Position", data.position);
//                telemetry.addData("Distance", data.distance + " cm\n");
//            }
//        }
//        previousDetectedObjects = new ArrayList<>(detectedObjects);  // Update cache


        telemetry.update();

    }

    @Override
    protected void init_loop_internal() {

    }

    @Override
    protected void init_internal() {

        // Vision
//             int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//                    camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WC1"), cameraMonitorViewId);
//
//                    visionPipeline = new FindRedSpecimen(telemetry);
//                    camera.setPipeline(visionPipeline);
//                    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                        @Override
//                        public void onOpened() {
//                            camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
//                        }
//
//                        @Override
//                        public void onError(int errorCode) {
//
//                        }
//                    });
//
//            FtcDashboard dashboard = FtcDashboard.getInstance();
//            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//            FtcDashboard.getInstance().startCameraStream(camera, 10);


    }
}
