package net.polarbub.ftc.tele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;

@TeleOp(name = "Main", group = MainBotConfig.opmodeGroupName)
public class MainTeleop extends FastIterativeOpMode {
    //
    // Dashboard options
    //
    public static long launcherVelocityCheckInterval = 200;

    //MUST BE IN ORDER OF DISTANCE!!!!!
    public static List<Pair<Double, Double>> launchSpeedMapping = Arrays.asList(
                new MutablePair<>(24.5, 1470.0),
                new MutablePair<>(28.5, 1480.0),
                new MutablePair<>(36.5, 1500.0),
                new MutablePair<>(42.0, 1540.0),
                new MutablePair<>(50.0, 1620.0),
                new MutablePair<>(57.7, 1620.0)
            );

    //
    // Options
    //
    //Offcolor lets us know that the blinkin is working when we press init
    private static final RevBlinkinLedDriver.BlinkinPattern offColor = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;

    private static final RevBlinkinLedDriver.BlinkinPattern manualLaunchControl = RevBlinkinLedDriver.BlinkinPattern.VIOLET;

    private static final RevBlinkinLedDriver.BlinkinPattern correctSpeed = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    private static final RevBlinkinLedDriver.BlinkinPattern speedingUpVisionLocked = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    private static final RevBlinkinLedDriver.BlinkinPattern outOfLaunchRange = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
    private static final RevBlinkinLedDriver.BlinkinPattern tagNotFoundPattern = RevBlinkinLedDriver.BlinkinPattern.RED;

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
    public Boolean visionDebugInfo = false;
    public Alliance alliance = Alliance.lastAlliance;


    private static double launchSpeedFromDist(double dist) {
        Pair<Double, Double> lowestPair = launchSpeedMapping.get(0);
        if(dist < lowestPair.getKey()) {
            return Double.NaN;
        }


        for(int i = 0; i < launchSpeedMapping.size(); i++) {
            Pair<Double, Double> p = launchSpeedMapping.get(i);
            if(dist == p.getKey()) {
                return p.getValue();
            }

            if(dist < p.getKey()) {
                //Previous one must have been greater, so we know we are between the two closest points,
                // so we interpolate with a weighted average
                Pair<Double, Double> last = launchSpeedMapping.get(i-1);
                double deltaDist = p.getKey() - last.getKey();
                double offset = dist - last.getKey();
                double weightLast = (deltaDist - offset) / deltaDist;
                double weightThis = offset / deltaDist;


                return weightLast * last.getValue() + weightThis * p.getValue();
            }
        }
        return Double.NaN;
    }

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
            optionDriver.add(new BooleanOption("Vision debug info", "dpad_up", gamepad1, this.getClass().getField("visionDebugInfo"), this));
            optionDriver.add(new EnumOption<>("Alliance", "cross", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        hardware.blinkin.setPattern(offColor);

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
                    power = 0;
                } else if(left_bumper) {
                    power = MainBotHardware.moverDrivePower;
                } else if(right_bumper) {
                    power = -MainBotHardware.moverDrivePower;
                } else {
                    throw new RuntimeException("Oopsy. Something bad happened. Never should be here");
                }

                if(hardware.kickerServo.getPosition() == 0 || power == 0) {
                    hardware.moverDrive.setPower(power);
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("options"), (Boolean options) -> {
                if(options) {
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoUpPos);
                    hardware.moverDrive.setPower(0);
                } else if (!gamepad1.circle){
                    hardware.kickerServo.setPosition(0);
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("triangle"), (Boolean cross) -> {
                if(cross && launcherControlMethod != LauncherControlMethod.MANUAL) {
                    runLauncher = !runLauncher;
                    if(runLauncher) {
                        hardware.launchDrive.setVelocity(MainBotHardware.teleOpLauncherNoVisionVelocity);
                    } else {
                        hardware.launchDrive.setVelocity(0);
                        hardware.blinkin.setPattern(MainBotHardware.wrongSpeedNoVisionPattern);
                    }
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("circle"), (Boolean circle) -> {
                if(circle) {
                    if(hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) < MainBotHardware.ballInKickerDist
                            && hardware.launchDrive.isAtTargetVelocity()
                            && hardware.launchDrive.getTargetVelocity() != 0
                    ) {
                        hardware.kickerServo.setPosition(MainBotHardware.kickerServoUpPos);
                        hardware.moverDrive.setPower(0);
                    }
                } else if(!gamepad1.options) {
                    hardware.kickerServo.setPosition(0);
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
                        hardware.blinkin.setPattern(MainBotHardware.wrongSpeedNoVisionPattern);
                        hardware.launchDrive.setVelocity(0);
                        //ADD: Switch away from vision control
                    } else if(runVision) {
                        launcherControlMethod = LauncherControlMethod.VISION;
                        hardware.blinkin.setPattern(tagNotFoundPattern);
                        hardware.launchDrive.setVelocity(0);
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
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        hardware.blinkin.setPattern(MainBotHardware.wrongSpeedNoVisionPattern);
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

        timedEvents.add(new TimedEvent(launcherVelocityCheckInterval, () -> {
            StringBuilder sb = new StringBuilder();
            sb.append("Launcher Control Mode: ").append(launcherControlMethod);
//            System.out.println("Run vision: " + runVision);
//            System.out.println("Run vision deb info: " + visionDebugInfo);

            double visionTargetSpeed = Double.NaN;
            double groundDist = Double.NaN;
            if(runVision) {
                ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();

                StringBuilder visionDebugSb = new StringBuilder();

                visionDebugSb.append("Target ID: ");
                visionDebugSb.append(aprilTagID).append('\n');

                AprilTagDetection targetDetection = null;
                if (detections != null && !detections.isEmpty()) {
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

                        groundDist = AprilTagDetectionPipeline.getGroundDistance(targetDetection.pose);
                        visionDebugSb.append("Ground distance: ").append(groundDist);
                        visionTargetSpeed = launchSpeedFromDist(groundDist);
                        visionDebugSb.append("Launcher Vision Target Velocity: ");
                        visionDebugSb.append(visionTargetSpeed).append("\n");

                    } else {
                        visionDebugSb.append("Target not found in frame\n");
                    }

                } else {
                    visionDebugSb.append("No detections\n");
                }

                visionDebugSb.append(pipeline.toString()).append('\n');

                if(visionDebugInfo) {
                    sb.append(visionDebugSb);
                }
            }

            launcherTelemetry = sb.toString();

            if(hardware.launchDrive.getTargetVelocity() != 0) {
                if(launcherControlMethod == LauncherControlMethod.VISION) {
                    if(Double.isNaN(groundDist)) {
                        hardware.launchDrive.setVelocity(MainBotHardware.teleOpLauncherNoVisionVelocity);
                        hardware.blinkin.setPattern(tagNotFoundPattern);

                    } else if(Double.isNaN(visionTargetSpeed)){
                        hardware.launchDrive.setVelocity(MainBotHardware.teleOpLauncherNoVisionVelocity);
                        hardware.blinkin.setPattern(outOfLaunchRange);

                    } else {
                        hardware.launchDrive.setVelocity(visionTargetSpeed);
                        if(hardware.launchDrive.isAtTargetVelocity()) {
                            hardware.blinkin.setPattern(correctSpeed);
                        } else {
                            hardware.blinkin.setPattern(speedingUpVisionLocked);
                        }
                    }

                } else if(launcherControlMethod == LauncherControlMethod.PRESET) {
                    if(hardware.launchDrive.isAtTargetVelocity()) {
                        hardware.blinkin.setPattern(MainBotHardware.correctSpeedNoVision);
                    } else {
                        hardware.blinkin.setPattern(MainBotHardware.wrongSpeedNoVisionPattern);
                    }
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

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start_internal() {
        if(runVision) {
            aprilTagID = alliance.toAprilTagID();
            pipeline = new AprilTagDetectionPipeline(hardware);
            hardware.blinkin.setPattern(tagNotFoundPattern);
            launcherControlMethod = LauncherControlMethod.VISION;
        } else {
            hardware.blinkin.setPattern(MainBotHardware.wrongSpeedNoVisionPattern);
            launcherControlMethod = LauncherControlMethod.PRESET;
        }

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    private long lastNanos = 0;
    private final long[] loopTimes = new long[5];
    private int loopTimesPos = 0;
    private String launcherTelemetry = "";

    public void loop_internal() {
        hardware.modules.clearBulkCache();

        //Check for changes of watched variables and run the handler if they did
        for(VariableWatcher variableWatcher : variablesWatched) {
            variableWatcher.tick();
        }

        long thisNanos = runtime.nanoseconds();
        for(TimedEvent event : timedEvents) {
            event.tick(thisNanos);
        }

        if(visionDebugInfo) {
            telemetry.addLine(launcherTelemetry);
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
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop_internal() {
        hardware.blinkin.setPattern(offColor);
    }
}
