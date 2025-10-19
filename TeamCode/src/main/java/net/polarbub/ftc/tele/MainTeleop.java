package net.polarbub.ftc.tele;

import android.util.Log;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.polarbub.ftc.MainBotConfig;
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

import java.util.ArrayList;
import java.util.List;

import static net.polarbub.ftc.MainBotConfig.MoveConfig;
import static java.lang.Math.abs;

import org.openftc.easyopencv.OpenCvCamera;


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
    private List<LynxModule> allHubs;
    private OptionDriver optionDriver;
    private List<VariableWatcher> variablesWatched = new ArrayList<>();
    private List<TimedEvent> timedEvents = new ArrayList<>();
    private OpenCvCamera launchCamera;

    //
    // Option Driver Options
    //
    public Boolean isPartyMode = false;
    public Alliance alliance = Alliance.lastAlliance;

    //
    // Code
    //

    private enum Mode {
        HOMING,
        HOMING_OUT,
        NORMAL,
        SPECIMEN,
        HANG;

        public RevBlinkinLedDriver.BlinkinPattern toColor() {
            switch (this) {
                case HOMING:
                    return RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                case HOMING_OUT:
                    return RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
                case NORMAL:
                    return RevBlinkinLedDriver.BlinkinPattern.GREEN;
                case SPECIMEN:
                    return RevBlinkinLedDriver.BlinkinPattern.BLUE;
                case HANG:
                    return RevBlinkinLedDriver.BlinkinPattern.RED;
            }
            throw new RuntimeException("Mode toColor() not complete");
        }
    }
    private Mode mode = Mode.NORMAL;

    private void setMode(Mode mode) {
        this.mode = mode;
        hardware.blinkin.setPattern(mode.toColor());
    }




    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init_internal() {
        hardware = new MainBotHardware(hardwareMap);
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new BooleanOption("'Party Mode'", "a", gamepad1, this.getClass().getField("isPartyMode"), this));
            optionDriver.add(new BooleanOption("Run Vision", "dpad_down", gamepad1, this.getClass().getField("isRunVision"), this));
            optionDriver.add(new EnumOption<>("Alliance", "dpad_up", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
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

                        hardware.leftFrontDrive.setPower(leftFrontPower);
                        hardware.leftRearDrive.setPower(leftBackPower);
                        hardware.rightFrontDrive.setPower(rightFrontPower);
                        hardware.rightRearDrive.setPower(rightBackPower);

                    }
            ));


        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        // Tell the driver that initialization is complete.
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
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

                    }
                    if(right_bumper) {

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
                        hardware.blinkin.setPattern(mode.toColor());
                        warningOn = false;
                    }
                    haveIncorrectColor = false;
                }
            }));

            timedEvents.add(new TimedEvent(incorrectColorFlashTime, () -> {
                if(haveIncorrectColor) {
                    if(warningOn) {
                        hardware.blinkin.setPattern(mode.toColor());
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

//    private int /

    private long lastNanos = 0;
    private final long[] loopTimes = new long[5];
    private int loopTimesPos = 0;

    public void loop_internal() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

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