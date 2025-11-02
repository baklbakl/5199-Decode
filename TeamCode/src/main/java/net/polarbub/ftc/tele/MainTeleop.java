package net.polarbub.ftc.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.auto.vision.AprilTagDetectionPipeline;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.lib.Alliance;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import net.polarbub.ftc.lib.TimedEvent;
import net.polarbub.ftc.lib.optiondriver.BooleanOption;
import net.polarbub.ftc.lib.optiondriver.EnumOption;
import net.polarbub.ftc.lib.optiondriver.OptionDriver;
import net.polarbub.ftc.lib.variablewatcher.DoubleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.SingleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.TripleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.VariableWatcher;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

@TeleOp(name = "Main", group = MainBotConfig.opmodeGroupName)
public class MainTeleop extends FastIterativeOpMode {
    //
    // Dashboard options
    //
    public static long launcherVelocityCheckInterval = 200;
    public static long launcherNoVisionVelocity = 1400;
    
    //
    // Options
    //
    //Offcolor lets us know that the blinkin is working when we press init
    private static final RevBlinkinLedDriver.BlinkinPattern offColor = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;

    private static final RevBlinkinLedDriver.BlinkinPattern manualLaunchControl = RevBlinkinLedDriver.BlinkinPattern.VIOLET;

    private static final RevBlinkinLedDriver.BlinkinPattern wrongSpeedNoVision = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
    private static final RevBlinkinLedDriver.BlinkinPattern correctSpeedNoVision = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;

    private static final RevBlinkinLedDriver.BlinkinPattern correctSpeed = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    private static final RevBlinkinLedDriver.BlinkinPattern speedingUpVisionLocked = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    private static final RevBlinkinLedDriver.BlinkinPattern outOfLaunchRange = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
    private static final RevBlinkinLedDriver.BlinkinPattern tagNotFound = RevBlinkinLedDriver.BlinkinPattern.RED;

    //
    //Globals
    //
    private final ElapsedTime runtime = new ElapsedTime();
    private MainBotHardware hardware = null;
    private OptionDriver optionDriver;
    private List<VariableWatcher> variablesWatched = new ArrayList<>();
    private List<TimedEvent> timedEvents = new ArrayList<>();

    private AprilTagDetectionPipeline pipeline;

    private Boolean runLauncher = false;
    private int aprilTagID;

    private enum LauncherControlMethod {
        MANUAL,
        VISION,
        PRESET
    }

    private LauncherControlMethod launcherControlMethod;



    //
    // Option Driver Options. MUST BE PUBLIC OR ELSE OPTION DRIVER CAN'T SEE IT.
    //
    public Boolean runVision = false;
    public Alliance alliance = Alliance.lastAlliance;


    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init_internal() {
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        hardware.modules.enableBulkCaching(LynxModule.BulkCachingMode.MANUAL);

        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new BooleanOption("Run Vision", "dpad_down", gamepad1, this.getClass().getField("runVision"), this));
            optionDriver.add(new EnumOption<>("Alliance", "dpad_up", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        hardware.blinkin.setPattern(offColor);
        launcherControlMethod = LauncherControlMethod.PRESET;
        RobotLog.ii(this.getClass().getSimpleName(), hardware.launchDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());

        //Control Scheme
        // Left shoulder - intake (black wheels)
        // Rigt shoulder - outake (black wheels)
        // triangle/y - toggle for launch motor
        // left stick - moving
        // right stick - moving (turn)
        // LEDs - distance to April Tag (based on alliance)

        try {
            //Movement
            variablesWatched.add(new TripleVariableWatcher<>(gamepad1, Gamepad.class.getDeclaredField("left_stick_x"), Gamepad.class.getDeclaredField("left_stick_y"), Gamepad.class.getDeclaredField("right_stick_x"), Float::equals,
                    (Float left_stick_x_f, Float left_stick_y_f, Float turning_f) -> {

                        // Note: pushing stick forward gives negative value
                        double leftFrontPower  = -left_stick_y_f + left_stick_x_f + turning_f;
                        double rightFrontPower = -left_stick_y_f - left_stick_x_f - turning_f;
                        double leftBackPower   = -left_stick_y_f - left_stick_x_f + turning_f;
                        double rightBackPower  = -left_stick_y_f + left_stick_x_f - turning_f;

                        // Normalize the values so no wheel power exceeds 100%
                        // This ensures that the robot maintains the desired motion.
                        double max = Math.max(Math.max(abs(leftBackPower), Math.max(abs(leftFrontPower), abs(rightFrontPower))), abs(rightBackPower));

                        if (max > 1.0) {
                            leftFrontPower  /= max;
                            rightFrontPower /= max;
                            leftBackPower   /= max;
                            rightBackPower  /= max;
                        }

                        hardware.leftFrontDrive.setPower(leftFrontPower);
                        hardware.leftRearDrive.setPower(leftBackPower);
                        hardware.rightFrontDrive.setPower(rightFrontPower);
                        hardware.rightRearDrive.setPower(rightBackPower);

                    }
            ));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_bumper"), Gamepad.class.getField("right_bumper"), Boolean::equals, (Boolean left_bumper, Boolean right_bumper) -> {
                double power;
                if(left_bumper == right_bumper) {
                    power = MainBotConfig.restingServoPos;
                } else if(left_bumper) {
                    power = 0;
                } else if(right_bumper) {
                    power = 1;
                } else {
                    throw new RuntimeException("Oopsy. Something bad happened. Never should be here");
                }

                hardware.moverServo.setPosition(power);
                hardware.moverServo2.setPosition(power);
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("y"), (Boolean y) -> {
                if(y && launcherControlMethod != LauncherControlMethod.MANUAL) {
                    runLauncher = !runLauncher;
                    if(runLauncher) {
                        hardware.launchDrive.setVelocity(launcherNoVisionVelocity);
                    } else {
                        hardware.launchDrive.setVelocity(0);
                        hardware.blinkin.setPattern(wrongSpeedNoVision);
                    }
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("dpad_up"), (Boolean dpad_up) -> {
                if(dpad_up) {
                    hardware.launchDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MainBotHardware.launcherCoefficients);
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_trigger"), Gamepad.class.getField("right_trigger"), Float::equals, (Float left_trigger, Float right_trigger) -> {
                float power = left_trigger - right_trigger;
                if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                    if(power > 0) {
                        power /= 2;
                    }
                    hardware.launchDrive.setPower(power);
                } else {
                    hardware.intakeDrive.setPower(power);
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_stick_button"), Boolean::equals, (Boolean left_stick_button) -> {
                if(left_stick_button) {
                    if(launcherControlMethod == LauncherControlMethod.VISION) {
                        launcherControlMethod = LauncherControlMethod.PRESET;
                        //ADD: Switch away from vision control
                    } else if(runVision) {
                        launcherControlMethod = LauncherControlMethod.VISION;
                        //ADD: Switch over to vision control
                    }
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("right_stick_button"), Boolean::equals, (Boolean right_stick_button) -> {
                if(right_stick_button) {
                    if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                        launcherControlMethod = LauncherControlMethod.PRESET;
                        hardware.intakeDrive.setPower(hardware.launchDrive.getPower());
                        hardware.launchDrive.setVelocity(0);
                        hardware.launchDrive.setPower(1);
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        hardware.blinkin.setPattern(wrongSpeedNoVision);
                    } else {
                        launcherControlMethod = LauncherControlMethod.MANUAL;
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hardware.launchDrive.setPower(hardware.intakeDrive.getPower());
                        hardware.intakeDrive.setPower(0);
                        hardware.blinkin.setPattern(manualLaunchControl);
                    }
                }
            }));

        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        if(runVision) {
            pipeline = new AprilTagDetectionPipeline(hardware);
        }

        timedEvents.add(new TimedEvent(launcherVelocityCheckInterval, () -> {
            if(launcherControlMethod == LauncherControlMethod.PRESET && hardware.launchDrive.getTargetVelocity() != 0) {
                if(hardware.launchDrive.isAtTargetVelocity()) {
                    hardware.blinkin.setPattern(correctSpeedNoVision);
                } else {
                    hardware.blinkin.setPattern(wrongSpeedNoVision);
                }
            }
        }));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop_internal() {
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
        } catch (InterruptedException e) {
        }
    }

//    private boolean warningOn;
//    private boolean haveIncorrectColor;

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start_internal() {
        if(runVision) {
            if(alliance == Alliance.BLUE) {
                aprilTagID = 20;
            } else {
                aprilTagID = 24;
            }
            hardware.blinkin.setPattern(tagNotFound);
        } else {
            hardware.blinkin.setPattern(wrongSpeedNoVision);
        }




        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    private long lastNanos = 0;
    private final long[] loopTimes = new long[5];
    private int loopTimesPos = 0;

//    private long startTime = -1;
//    private boolean lastVelocityZero = false;

    public void loop_internal() {
        if(runVision) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();

            telemetry.addData("Target ID", aprilTagID);
            if (detections != null && !detections.isEmpty()) {
                AprilTagDetection best = null;
                for (AprilTagDetection d : detections) {
                    if (d.id == aprilTagID) {
                        best = d;
                        break;
                    }
                }

                telemetry.addData("Detections", detections.size());
                if (best != null) {
                    telemetry.addLine("Target Found!");
                    telemetry.addData("ID", best.id);
                    telemetry.addData("x (m)", "%.3f", best.pose.x);
                    telemetry.addData("y (m)", "%.3f", best.pose.y);
                    telemetry.addData("z (m)", "%.3f", best.pose.z);
                } else {
                    telemetry.addLine("Target not found in frame");
                }
            } else {
                telemetry.addLine("No detections");
            }
        }

        hardware.modules.clearBulkCache();

        //Check for changes of watched variables and run the handler if they did
        for(VariableWatcher variableWatcher : variablesWatched) {
            variableWatcher.tick();
        }

        long thisNanos = runtime.nanoseconds();
        for(TimedEvent event : timedEvents) {
            event.tick(thisNanos);
        }

        //Average the last five iterations of the loop to minimize jumping around
        loopTimes[loopTimesPos] = thisNanos - lastNanos;
        loopTimesPos++;
        loopTimesPos %= loopTimes.length;
        long averageNanos = 0;
        for(long time : loopTimes) {
            averageNanos += time;
        }
        averageNanos /= loopTimes.length;
        lastNanos = thisNanos;

        telemetry.addData("Status",
                "Loop Speed: " + 1.0d/(averageNanos/1000000000.0d) + " Hz\n" +
                        "Loop Time: " + averageNanos / 1000000000.0d + "s\n"
        );
        telemetry.update();

//        TelemetryPacket packet = new TelemetryPacket();
//        double velocity = hardware.launchDrive.getVelocity();
//        double targetVelocity;
//
//        if(hardware.launchDrive.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
//            targetVelocity = hardware.launchDrive.getTargetVelocity();
//            if(targetVelocity != 0 && lastVelocityZero && startTime == -1) {
//                startTime = thisNanos;
//            }
//            lastVelocityZero = targetVelocity == 0;
//
//        } else {
//            targetVelocity = launcherNoVisionVelocity;
//
//            if(velocity != 0 && lastVelocityZero) {
//                RobotLog.ii("MainTeleOp", "Starting");
//                startTime = thisNanos;
//            }
//            lastVelocityZero = velocity == 0;
//        }
//
//        if(startTime != -1 && abs(velocity) >= abs(targetVelocity)) {
//            RobotLog.ii("MainTeleOp", (thisNanos - startTime) / 1000000000.0d + " seconds to spin up");
//            startTime = -1;
//        }
//
//        packet.put("LauncherVelocity", velocity);
//        packet.put("Target", targetVelocity);
//        packet.put("Min", 0);
//        packet.put("Max", 2000);
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop_internal() {
        hardware.blinkin.setPattern(offColor);
    }
}
