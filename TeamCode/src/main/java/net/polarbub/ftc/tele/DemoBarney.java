package net.polarbub.ftc.tele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import net.polarbub.ftc.lib.TimedEvent;
import net.polarbub.ftc.lib.variablewatcher.DoubleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.SingleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.TripleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.VariableWatcher;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

@TeleOp(name = "DemoBot (Big)", group = MainBotConfig.demoGroupName)
//@Config
public class DemoBarney extends FastIterativeOpMode {
    //
    // Dashboard options
    //
    public static double speedMultiplier = 0.5;

    //
    // Options
    //

    //[0,1] the point that the dead zone removal starts the motors
    private static final RevBlinkinLedDriver.BlinkinPattern offColor = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;

    //
    //Globals
    //
    private final ElapsedTime runtime = new ElapsedTime();
    private MainBotHardware hardware = null;
    private List<VariableWatcher> variablesWatched = new ArrayList<>();
    private List<TimedEvent> timedEvents = new ArrayList<>();

    //
    // Code
    //

    private enum Mode {
        HOMING,
        NORMAL;

        public RevBlinkinLedDriver.BlinkinPattern toColor() {
            switch (this) {
                case HOMING:
                    return RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                case NORMAL:
                    return RevBlinkinLedDriver.BlinkinPattern.GREEN;
            }
            throw new RuntimeException("Mode toColor() not complete");
        }
    }
    private Mode mode = Mode.NORMAL;

    private void setMode(Mode mode) {
        this.mode = mode;
        hardware.blinkin.setPattern(mode.toColor());
    }

    private void startHoming() {
        setMode(Mode.HOMING);
        hardware.pivotDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.pivotDrive.setPower(ArmConfig.pivotHomingPower);
        hardware.slideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.slideDrive.setPower(ArmConfig.slideHomingPower);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init_internal() {
        hardware = new MainBotHardware(hardwareMap);
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        hardware.modules.enableBulkCaching(LynxModule.BulkCachingMode.MANUAL);

        hardware.blinkin.setPattern(offColor);

        //Control Scheme
        //Left stick        - movement
        //Left stick button - enter homing mode
        //Right stick       - rotate
        //Dpad up           - move pivot up
        //Dpad down         - move pivot down
        //Y                 - move slides out
        //A                 - mode slides in
        //Left shoulder     - Intake in
        //Right shoulder    - Intake out
        //X                 - Enter / exit specimen mode
        //Dpad right + B    - Enter / exit hang mode
        //Left trigger      - Hang retract slides
        //Right trigger     - hand extend slides

        try {
            //Movement
            variablesWatched.add(new TripleVariableWatcher<>(
                    gamepad1,
                    Gamepad.class.getDeclaredField("left_stick_x"),
                    Gamepad.class.getDeclaredField("left_stick_y"),
                    Gamepad.class.getDeclaredField("right_stick_x"),
                    Float::equals,
                    (Float left_stick_x_f, Float left_stick_y_f, Float turning_f) -> {
                        /*Double left_stick_x = Double.valueOf(left_stick_x_f) + 0.0;
                        Double left_stick_y = -Double.valueOf(left_stick_y_f) + 0.0;
                        double turning = Double.valueOf(turning_f) + 0.0;
                        //Use trig to get the heading. Because we use trig forward is 90 degrees, right is 0, and left is 180. This is like the unit circle
                        //See https://www.desmos.com/calculator/oobhklsqt7
                        short heading = (short) round(atan(left_stick_y / left_stick_x) * 180.0 / PI);
                        //Fix that tangent is only Q1 and Q4
                        //In Q2 and Q3 180 degrees are added
                        if (left_stick_x < 0) {
                            heading += 180;
                        }
                        //In Q4 360 degrees are added
                        if(left_stick_x >= 0 && left_stick_y < 0) {
                            heading += 360;
                        }

                        //heading = (short) ((heading + 360) % 360);
//                        System.out.println("Heading:" + heading + "\nX: " + left_stick_x + "\nY: " + left_stick_y + "\n");

                        //No conversion to the turning joystick as it is already [-1,1] (Interval notation, thanks Medansky) and that works when you multiply it by the
                        //wheel speed.
                        //Get rid of the dead spot
                        if (turning != 0) {
                            turning *= inverseMotorStartPower;
                            turning += motorStartPower;
                        }
                        turning /= 2;
                        //System.out.println("Turning: " + turning);

                        //This uses pythagorean theorem to determine how far the stick is from the center aka the requested speed.
                        double speed = sqrt((left_stick_x * left_stick_x) + (left_stick_y * left_stick_y));
                        //Get rid of the dead spot
                        if (speed != 0) {
                            speed *= inverseMotorStartPower;
                            speed += motorStartPower;
                        }
                        //System.out.println("Speed: " + speed);

                        //And cap the speed if we ever get over 1.
                        if (speed > 1) speed = 1;

                        //This does \frac{\left|\operatorname{mod}\left(h-45,360\right)-180\right|-90}{90} Paste it in Desmos
                        //|mod(heading - 45, 360) - 180| - 90
                        //-----------------------------
                        //             90
                        //It is a triangle wave to convert the heading [0,360) into the max wheel speed [-1,1].
                        //There is only two speeds even though we have four wheels because wheels that are diagonal to each other spin the same speed
                        double leftSpeed = (abs(((heading - 45 + 360) % 360) - 180) - 90) / 90.0;
                        double rightSpeed = (abs(((heading - 135 + 360) % 360) - 180) - 90) / 90.0;
                        rightSpeed *= speed;
                        leftSpeed *= speed;
//                        System.out.println("Heading: " + heading + "\nLeft speed:" + leftSpeed + "\nRight speed: " + rightSpeed);
                        //Send calculated power to wheels
                        mainBotHardware.leftFrontDrive.setPower(leftSpeed + turning > 1 ? 1 : leftSpeed + turning);
                        mainBotHardware.leftRearDrive.setPower(rightSpeed + turning > 1 ? 1 : rightSpeed + turning);
                        mainBotHardware.rightFrontDrive.setPower(rightSpeed - turning > 1 ? 1 : rightSpeed - turning);
                        mainBotHardware.rightRearDrive.setPower(leftSpeed - turning > 1 ? 1 : leftSpeed - turning);*/


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

                        hardware.leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
                        hardware.leftRearDrive.setPower(leftBackPower * speedMultiplier);
                        hardware.rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
                        hardware.rightRearDrive.setPower(rightBackPower * speedMultiplier);

                    }
            ));

            variablesWatched.add(new SingleVariableWatcher<>(
                    gamepad1,
                    Gamepad.class.getField("left_stick_button"),
                    Boolean::equals,
                    (Boolean left_stick_button) -> {
                        if(left_stick_button) {
                            if(mode == Mode.NORMAL) {
                                startHoming();
                            } else if(mode == Mode.HOMING) {
                                setMode(Mode.NORMAL);
                                hardware.pivotDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                hardware.slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                hardware.pivotDrive.runMotorToPos(ArmConfig.restingPivotCounts, ArmConfig.pivotPower);
                                hardware.slideDrive.runMotorToPos(ArmConfig.restingSlideCounts, ArmConfig.slidePower);
                            }
                        }
            }));

            //Pivot presets
            variablesWatched.add(new DoubleVariableWatcher<>(
                    gamepad1,
                    Gamepad.class.getField("dpad_up"),
                    Gamepad.class.getField("dpad_down"),
                    Boolean::equals,
                    (Boolean dpad_up, Boolean dpad_down) -> {
                        if(dpad_up && dpad_down) {
                            return;
                        }

                        if(mode == Mode.NORMAL) {
                            int pivotTarget = hardware.pivotDrive.getTargetPosition();
                            int slideTarget = hardware.slideDrive.getTargetPosition();

                            if(dpad_up) {
                                if(pivotTarget == ArmConfig.restingPivotCounts) {
                                    hardware.pivotDrive.setTargetPosition(ArmConfig.hoverPivotCounts);
                                }

                            } else if(dpad_down) {
                                if(pivotTarget == ArmConfig.hoverPivotCounts) {
                                    if(slideTarget == ArmConfig.restingSlideCounts) {
                                        hardware.pivotDrive.setTargetPosition(ArmConfig.restingPivotCounts);
                                    }
                                }
                            }
                        }
            }));

            //Run grabber
            variablesWatched.add(new DoubleVariableWatcher<>(
                    gamepad1,
                    Gamepad.class.getField("left_bumper"),
                    Gamepad.class.getField("right_bumper"),
                    Boolean::equals,
                    (Boolean left_bumper, Boolean right_bumper) -> {
                        if (right_bumper) {
                            double servoPos;
                            if(hardware.pivotDrive.getTargetPosition() == ArmConfig.basketPivotCounts && hardware.slideDrive.getTargetPosition() == ArmConfig.basketSlideCounts) {
                                servoPos = ArmConfig.dropServoPos;
                            } else {
                                servoPos = ArmConfig.expelServoPos;
                            }
                            hardware.intakeServo.setPosition(servoPos);
                            hardware.intakeServo2.setPosition(1 - servoPos);
                        } else if (left_bumper && hardware.colorSensor.getDistance(DistanceUnit.INCH) > MainBotConfig.distSensorThresh) {
                            hardware.intakeServo.setPosition(ArmConfig.grabServoPos);
                            hardware.intakeServo2.setPosition(1-ArmConfig.grabServoPos);
                        } else {
                            hardware.intakeServo.setPosition(ArmConfig.restingServoPos);
                            hardware.intakeServo2.setPosition(1-ArmConfig.restingServoPos);
                        }

            }));

        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        timedEvents.add(new TimedEvent(30, () -> {
            if(hardware.intakeServo.getPosition() == ArmConfig.grabServoPos && hardware.colorSensor.getDistance(DistanceUnit.INCH) < MainBotConfig.distSensorThresh) {
                hardware.intakeServo.setPosition(ArmConfig.restingServoPos);
                hardware.intakeServo2.setPosition(1-ArmConfig.restingServoPos);
            }
        }));
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop_internal() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start_internal() {
        startHoming();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

//    private int /

    private long lastNanos = 0;
    private final long[] loopTimes = new long[5];
    private int loopTimesPos = 0;

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

        telemetry.addLine("Mode: " + mode);
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

