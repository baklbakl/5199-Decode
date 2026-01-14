package org.ducktapeandaprayer.ftc.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
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

        if (detections != null && !detections.isEmpty()) {
            telemetry.addData("Detections", detections.size());
            for (AprilTagDetection d : detections) {
                telemetry.addLine("Target Found!");
                telemetry.addLine(AprilTagDetectionPipeline.aprilTagDetectionToString(d));
//                telemetry.addData("groundDistance (in)", Math.cos(Math.atan(d.pose.y / d.pose.z)) * d.pose.z);
                telemetry.addData("groundDistance (in)", AprilTagDetectionPipeline.getGroundDistance(d.pose));
            }
        } else {
            telemetry.addLine("No detections");
        }
        telemetry.addLine(pipeline.getTimingTelemetry());
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
