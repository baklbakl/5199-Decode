package net.polarbub.ftc.tele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.auto.vision.opencv.AprilTagDetectionPipeline;
import net.polarbub.ftc.MainBotHardware;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp(name = "Main", group = MainBotConfig.opmodeGroupName)
//@Config
public class MainTeleop extends FastIterativeOpMode {
    //
    // Dashboard options
    //
    public static long partyPatternTime = 500;

    public static long colorSensorCheckInterval = 100;
    public static long incorrectColorFlashTime = 500;


    //
    // Options
    //

    //[0,1] the point that the dead zone removal starts the motors
    private static final RevBlinkinLedDriver.BlinkinPattern offColor = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;
    private static final RevBlinkinLedDriver.BlinkinPattern wrongArtifact = RevBlinkinLedDriver.BlinkinPattern.RED;




    //
    //Globals
    //
    private final ElapsedTime runtime = new ElapsedTime();
    private MainBotHardware hardware = null;
    private OptionDriver optionDriver;
    private List<VariableWatcher> variablesWatched = new ArrayList<>();
    private List<TimedEvent> timedEvents = new ArrayList<>();

    private AprilTagDetectionPipeline pipeline;
    private OpenCvCamera launchCamera;

    //
    // Option Driver Options
    //
    private Boolean isPartyMode = false;
    private Boolean isRunVision = false;
    private int aprilTagID;

    public Alliance alliance = Alliance.lastAlliance;


    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init_internal() {
        hardware = new MainBotHardware(hardwareMap);
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        hardware.modules.enableBulkCaching(LynxModule.BulkCachingMode.MANUAL);

        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new BooleanOption("'Party Mode'", "a", gamepad1, this.getClass().getField("isPartyMode"), this));
            optionDriver.add(new BooleanOption("Run Vision", "dpad_down", gamepad1, this.getClass().getField("isRunVision"), this));
            optionDriver.add(new EnumOption<>("Alliance", "dpad_up", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        if(alliance == Alliance.BLUE) {
            aprilTagID = 20;
        } else {
            aprilTagID = 24;
        }

        hardware.blinkin.setPattern(offColor);



        //Control Scheme
        // Left shoulder - intake (black wheels)
        // Rigt shoulder - outake (black wheels)
        // triangle/y - release servo to shoot artifact
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


        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

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

        if(!isPartyMode && isRunVision) {
            launchCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WC1"));
            pipeline = new AprilTagDetectionPipeline();

            launchCamera.setPipeline(pipeline);
            launchCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    launchCamera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
        }

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
        }
    }

    private RevBlinkinLedDriver.BlinkinPattern partyCurrentPattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(0);
    private boolean warningOn;
    private boolean haveIncorrectColor;

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start_internal() {
        if (isPartyMode) {
            hardware.blinkin.setPattern(partyCurrentPattern);
            timedEvents.add(new TimedEvent(partyPatternTime, () -> {
                hardware.blinkin.setPattern(partyCurrentPattern);
                partyCurrentPattern = partyCurrentPattern.next();
            }));
        } else {
            try {

                //Control Scheme
                // Left shoulder - intake (black wheels)
                // Rigt shoulder - outake (black wheels)
                // triangle/y - release servo to shoot artifact
                // left stick - moving
                // right stick - moving (turn)
                // LEDs - distance to April Tag (based on alliance)




                variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_bumper"), Gamepad.class.getField("right_bumper"), Boolean::equals, (Boolean left_bumper, Boolean right_bumper) -> {
                    if(left_bumper && right_bumper) {
                        return;
                    }

                    if(left_bumper) {
                        // TODO add to MainBotConfig
                        hardware.intakeDrive.setPower(0.8);

                    }
                    if(right_bumper) {
                        // TODO add to MainBotConfig
                        hardware.intakeDrive.setPower(-0.8);
                    }

                }));

                variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("y"), (Boolean y) -> {
                    if(y) {
                        hardware.launchServo.setPosition(1);
                        hardware.launchServo2.setPosition(1);

                    }
                }));


            } catch (NoSuchFieldException e) {
                throw new RuntimeException(e);
            }


            timedEvents.add(new TimedEvent(colorSensorCheckInterval, () -> {
                int redDetected = hardware.colorSensor.red();
                int blueDetected = hardware.colorSensor.blue();
                int greenDetected = hardware.colorSensor.green();

                if (
                        ((redDetected > blueDetected && redDetected > greenDetected) && alliance == Alliance.BLUE) ||
                                ((blueDetected > redDetected && blueDetected > greenDetected) && alliance == Alliance.RED)
                ) {
                    haveIncorrectColor = true;
                } else {
                    if (haveIncorrectColor && warningOn) {
//                        hardware.blinkin.setPattern(mode.toColor());
                        warningOn = false;
                    }
                    haveIncorrectColor = false;
                }
            }));

            timedEvents.add(new TimedEvent(incorrectColorFlashTime, () -> {
                if(haveIncorrectColor) {
                    if(warningOn) {
//                        hardware.blinkin.setPattern(mode.toColor());
                    } else {
                        hardware.blinkin.setPattern(wrongArtifact);
                    }
                    warningOn = !warningOn;
                }
            }));
        }

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */


    private long lastNanos = 0;
    private final long[] loopTimes = new long[5];
    private int loopTimesPos = 0;

    public void loop_internal() {

        if(isRunVision) {
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

        telemetry.addLine("Mode:");
        telemetry.addData("Status",
                "Loop Speed: " + 1.0d/(averageNanos/1000000000.0d) + " Hz\n" +
                        "Loop Time: " + averageNanos / 1000000000.0d + "s\n"
        );

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop_internal() {
        hardware.blinkin.setPattern(offColor);
    }
}
