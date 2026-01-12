package org.ducktapeandaprayer.ftc.tele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.auto.RobotImpl;
import org.ducktapeandaprayer.ftc.auto.SplitAuto;
import org.ducktapeandaprayer.ftc.auto.vision.AprilTagDetectionPipeline;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name = "Test", group = MainBotConfig.opmodeGroupName)
public class Test extends FastIterativeOpMode {
    public static double thresh = 0.8;

    private ElapsedTime runtime = new ElapsedTime();
    private MainBotHardware hardware;

    private AprilTagDetectionPipeline pipeline;

    public void init_internal() {
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        hardware.modules.enableBulkCaching(LynxModule.BulkCachingMode.MANUAL);
    
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        pipeline = new AprilTagDetectionPipeline(MainBotHardware.getOrCreate(hardwareMap));

    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop_internal() {
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

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start_internal() {
        runtime.reset();
        RobotImpl robot = new RobotImpl(hardware, this::opModeIsActive);
        robot.launch(3, SplitAuto.launchVelocity);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop_internal() {
//        hardware.modules.clearBulkCache();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop_internal() {
    }
}
