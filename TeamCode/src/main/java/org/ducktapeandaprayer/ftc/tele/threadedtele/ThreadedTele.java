package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.vision.AprilTagDetectionPipeline;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.LoopTimeCalculator;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.vision.AprilTagPoseEx;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "1ThreadedTele", group = MainBotConfig.opmodeGroupName)
public class ThreadedTele extends FastIterativeOpMode {
    public MainBotHardware hardware = null;
    private ControlHubThread controlHubThread = null;
    private ExpansionHubThread exansionHubThread = null;
    private Map<String, ThreadEx> threads = new HashMap<>();
    private LoopTimeCalculator timer = new LoopTimeCalculator();
    private OptionDriver optionDriver;
    private AprilTagDetectionPipeline pipeline;

    //Never change these after the threads start
    public volatile Boolean runVision = true;
    public volatile Boolean visionDebugInfo = true;
    public volatile Alliance alliance = Alliance.lastAlliance;
    public volatile Boolean autoAimOnAutoLaunch = false;
//    public volatile Boolean autoAimDebInfo = false;
//    public volatile Boolean autoSpeedDebInfo = false;
    public Boolean timingDebugInfo = false;

    private AprilTagPoseEx tagPose = null;
    public final Object tagPoseLock = new Object();

    public AprilTagPoseEx getTagPose() {
        return tagPose;
    }

    private String visionTelemetry = "";

    private void updateDetection() {
        if(!runVision) {
            return;
        }

        ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();
        if(detections == null) {
            return;
        }

        StringBuilder visionDebugSb = new StringBuilder();
        int aprilTagID = alliance.toAprilTagID();

        visionDebugSb.append("Target ID: ");
        visionDebugSb.append(aprilTagID).append('\n');

        AprilTagDetection targetDetection = null;
        if (!detections.isEmpty()) {
            for (AprilTagDetection d : detections) {
                if (d.id == aprilTagID) {
                    targetDetection = d;
                    break;
                }
            }

            visionDebugSb.append("Detected ").append(detections.size()).append("april tags\n");
            if (targetDetection != null) {
                visionDebugSb.append("Target Found!\n");
                visionDebugSb.append(AprilTagDetectionPipeline.aprilTagDetectionToString(targetDetection)).append('\n');
                synchronized (tagPoseLock) {
                    tagPose = new AprilTagPoseEx(targetDetection.pose);
                    visionDebugSb.append("Ground distance: ").append(tagPose.groundDistance).append("\n");
                }

            } else {
                visionDebugSb.append("Target not found in frame\n");
                synchronized (tagPoseLock) {
                    tagPose = null;
                }
            }

        } else {
            visionDebugSb.append("No detections\n");
            synchronized (tagPoseLock) {
                tagPose = null;
            }
        }

        visionDebugSb.append(pipeline.getTimingTelemetry()).append('\n');

        if(visionDebugInfo) {
            visionTelemetry = visionDebugSb.toString();
        }
    }

    @Override
    protected void init_internal() {
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        pipeline = new AprilTagDetectionPipeline(hardware);
        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new BooleanOption("Loop Timing debug info", "triangle", gamepad1, this.getClass().getField("timingDebugInfo"), this));
            optionDriver.add(new BooleanOption("Run Vision", "dpad_down", gamepad1, this.getClass().getField("runVision"), this));
            optionDriver.add(new BooleanOption("Vision debug info", "dpad_up", gamepad1, this.getClass().getField("visionDebugInfo"), this));
            optionDriver.add(new EnumOption<>("Alliance", "cross", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        controlHubThread = new ControlHubThread(hardware, this, gamepad1, optionDriver);
        exansionHubThread = new ExpansionHubThread(hardware, this, controlHubThread, gamepad1, optionDriver);

        threads.put("control hub thread", controlHubThread);
        threads.put("expansion hub thread", exansionHubThread);
    }

    @Override
    protected void init_loop_internal() {
        String out = optionDriver.tick();
        if(out != null) {
            telemetry.addLine(out);
            telemetry.update();

            if (alliance == Alliance.BLUE) {
                hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (alliance == Alliance.RED) {
                hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        }

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {}
    }

    @Override
    protected void start_internal() {
        pipeline = new AprilTagDetectionPipeline(hardware);

        for(ThreadEx t : threads.values()) {
            t.start();
        }

        timer.reset();
    }

    @Override
    protected void loop_internal() {
        try {
            updateDetection();

            for(Map.Entry<String, ThreadEx> element : threads.entrySet()) {
                ThreadEx thread = element.getValue();
                Throwable err = thread.getErr();
                if(err != null) {
                    //Clean up after ourselves
                    stop_internal();
                    RobotLog.ii("ThreadedTele", "Stopping due to error");
                    throw new RuntimeException("Error in " + element.getKey(), err);
                }

                thread.addTelemetryTo(telemetry);
                thread.feedWatchdog();
            }

            telemetry.addLine();
            telemetry.addLine(visionTelemetry);

            if(timingDebugInfo) {
                for(Map.Entry<String, ThreadEx> t : threads.entrySet()) {
                    telemetry.addData("Timings for " + t.getKey(), t.getValue().loopTimes);
                }

                telemetry.addData("Main Loop Timings", timer.toString());
            }

            telemetry.update();

            timer.tick();

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        } catch (Exception e) {
            stop_internal();
            throw e;
        }
    }

    @Override
    protected void stop_internal() {
        for(ThreadEx t : threads.values()) {
            t.softStop();
        }

        for(ThreadEx t : threads.values()) {
            t.stopAndJoin();
        }
    }
}
