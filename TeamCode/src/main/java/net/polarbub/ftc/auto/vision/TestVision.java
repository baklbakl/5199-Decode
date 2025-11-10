package net.polarbub.ftc.auto.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "Test Vision")
public class TestVision extends FastIterativeOpMode {
    private AprilTagDetectionPipeline pipeline;

    @Override
    protected void init_internal() {
        pipeline = new AprilTagDetectionPipeline(MainBotHardware.getOrCreate(hardwareMap));
    }

    @Override
    protected void init_loop_internal() {
        ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();

        telemetry.addData("Target ID", 24);
        if (detections != null && !detections.isEmpty()) {
            telemetry.addData("Detections", detections.size());
            for (AprilTagDetection d : detections) {
                telemetry.addLine("Target Found!");
                telemetry.addData("ID", d.id);
                telemetry.addData("x (m)", "%.3f", d.pose.x);
                telemetry.addData("y (m)", "%.3f", d.pose.y);
                telemetry.addData("z (m)", "%.3f", d.pose.z);
            }

        } else {
            telemetry.addLine("No detections");
        }
        telemetry.addData("FPS", pipeline.camera.getFps());
        telemetry.addData("Ms per frame processing", pipeline.camera.getPipelineTimeMs());
        telemetry.addData("Ms per frame overhead", pipeline.camera.getOverheadTimeMs());
        telemetry.addData("Ms per frame total", pipeline.camera.getTotalFrameTimeMs());
        telemetry.update();

    }

    @Override
    protected void start_internal() {

    }

    @Override
    protected void loop_internal() {
    }

    @Override
    protected void stop_internal() {

    }
}
