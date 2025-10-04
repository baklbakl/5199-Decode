package net.polarbub.ftc.tele;

import android.util.Log;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.MainBotHardware;
import net.polarbub.ftc.auto.vision.opencv.FindNumSamples;
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

import static net.polarbub.ftc.MainBotConfig.ArmConfig;
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
    public static int sampleDetectedAlpha = 50;
    public static float redMinSaturation = 330;
    public static float redMaxSaturation = 10;
    public static float blueMinSaturation = 210;
    public static float blueMaxSaturation = 260;

    //
    // Options
    //

    //[0,1] the point that the dead zone removal starts the motors
    private static final double motorStartPower = 0.05;
    private static final RevBlinkinLedDriver.BlinkinPattern offColor = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;
    private static final RevBlinkinLedDriver.BlinkinPattern wrongSample = RevBlinkinLedDriver.BlinkinPattern.RED;


    //
    // Final Variables
    //
    private static final double inverseMotorStartPower = 1-motorStartPower;

    //
    //Globals
    //
    private final ElapsedTime runtime = new ElapsedTime();
    private MainBotHardware hardware = null;
    private List<LynxModule> allHubs;
    private OptionDriver optionDriver;
    private List<VariableWatcher> variablesWatched = new ArrayList<>();
    private List<TimedEvent> timedEvents = new ArrayList<>();
    private OpenCvCamera camera;
    private FindNumSamples pipeline;

    //
    // Option Driver Options
    //
    public Boolean isPartyMode = false;
    public Boolean isRunVision = false;
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

    private enum HomingDirection {
        IN,
        OUT,
    }

    private void startHoming(HomingDirection homingDirection) {
        double negate = 0;
        if(homingDirection == HomingDirection.IN) {
            setMode(Mode.HOMING);
            hardware.pivotDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.pivotDrive.setPower(ArmConfig.pivotHomingPower);
            negate = 1;
        } else if(homingDirection == HomingDirection.OUT) {
            hardware.pivotDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MainBotHardware.runMotorToPos(hardware.pivotDrive, ArmConfig.basketPivotCounts, ArmConfig.pivotPower);
            setMode(Mode.HOMING_OUT);
            negate = -1.5;
        }

        hardware.slideDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.slideDrive.setPower(ArmConfig.slideHomingPower * negate);
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

        if(!isPartyMode && isRunVision) {

            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WC1"));
            pipeline = new FindNumSamples();

            camera.setPipeline(pipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
        }

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

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_stick_button"), Boolean::equals, (Boolean left_stick_button) -> {
                if(left_stick_button) {
                    if(mode == Mode.NORMAL || mode == Mode.HOMING_OUT) {
                        startHoming(HomingDirection.IN);
                    } else if(mode == Mode.HOMING) {
                        setMode(Mode.NORMAL);
                        hardware.pivotDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        hardware.slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        MainBotHardware.runMotorToPos(hardware.pivotDrive, ArmConfig.restingPivotCounts, ArmConfig.pivotPower);
                        MainBotHardware.runMotorToPos(hardware.slideDrive, ArmConfig.restingSlideCounts, ArmConfig.slidePower);
                    }
                }
            }));
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
                if(isRunVision) {
                        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WC1"));
                        pipeline = new FindNumSamples();

                        camera.setPipeline(pipeline);
                        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                            @Override
                            public void onOpened() {
                                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                            }

                            @Override
                            public void onError(int errorCode) {

                            }
                        });

                    // Watch for Vision button only if the option is enabled
                    variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("options"), Boolean::equals, (Boolean b) -> {
                        if (b) {
                           int numSamples = 0;

                               if (alliance == Alliance.RED) {
                                   numSamples = pipeline.getRedCount();
                               } else if (alliance == Alliance.BLUE) {
                                   numSamples = pipeline.getBlueCount();
                               }



                            if(numSamples < 2) {
                               gamepad1.setLedColor(1, 0, 0, 2000);
                           } else if(numSamples >= 2 && numSamples <= 3) {
                               gamepad1.setLedColor(1, 1, 0, 2000);
                           } else if(numSamples >= 4) {
                               gamepad1.setLedColor(0, 1, 0, 2000);
                           } else {
                               gamepad1.setLedColor(1, 1, 1, 2000);
                           }
                        }
                    }));
                }

                //Pivot presets
                variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("dpad_up"), Gamepad.class.getField("dpad_down"), Boolean::equals, (Boolean dpad_up, Boolean dpad_down) -> {
                    if(dpad_up && dpad_down) {
                        return;
                    }

                    int pivotTarget = hardware.pivotDrive.getTargetPosition();
                    int slideTarget = hardware.slideDrive.getTargetPosition();

                    if(mode == Mode.NORMAL) {
                        if(dpad_up) {
                            if(pivotTarget == ArmConfig.restingPivotCounts) {
                                if(slideTarget != ArmConfig.restingSlideCounts) {
                                    throw new RuntimeException("Not allowed to have slides extended and pivot on peg");
                                }
                                hardware.pivotDrive.setTargetPosition(ArmConfig.hoverPivotCounts);
                            } else if(pivotTarget == ArmConfig.farIntakePivotCounts || pivotTarget == ArmConfig.closeIntakePivotCounts) {
                                if(slideTarget == ArmConfig.restingSlideCounts) {
                                    throw new RuntimeException("Not allowed to be in intake pivot position and slides retracted");
                                }
                                hardware.pivotDrive.setTargetPosition(ArmConfig.hoverPivotCounts);

                            } else if(pivotTarget == ArmConfig.hoverPivotCounts && slideTarget == ArmConfig.restingSlideCounts) {
                                hardware.pivotDrive.setTargetPosition(ArmConfig.basketPivotCounts);
                            }

                        } else if(dpad_down) {
                            if(pivotTarget == ArmConfig.basketPivotCounts && slideTarget == ArmConfig.restingSlideCounts) {
                                hardware.pivotDrive.setTargetPosition(ArmConfig.hoverPivotCounts);

                            } else if(pivotTarget == ArmConfig.hoverPivotCounts) {
                                if(slideTarget == ArmConfig.restingSlideCounts) {
                                    hardware.pivotDrive.setTargetPosition(ArmConfig.restingPivotCounts);
                                } else if (slideTarget == ArmConfig.farIntakeSlideCounts) {
                                    hardware.pivotDrive.setTargetPosition(ArmConfig.farIntakePivotCounts);
                                } else if (slideTarget == ArmConfig.closeIntakeSlideCounts) {
                                    hardware.pivotDrive.setTargetPosition(ArmConfig.closeIntakePivotCounts);
                                } else {
                                    throw new RuntimeException("How did we get here?");
                                }

                            }
                        }
                    } else if(mode == Mode.SPECIMEN) {
                        if(dpad_up && pivotTarget == ArmConfig.closeIntakePivotCounts) {
                            hardware.pivotDrive.setTargetPosition(ArmConfig.specimenPivotCounts);
                            hardware.slideDrive.setTargetPosition(ArmConfig.specimenApproachSlideCounts);
                        } else if(dpad_down && pivotTarget == ArmConfig.specimenPivotCounts) {
                            hardware.pivotDrive.setTargetPosition(ArmConfig.closeIntakePivotCounts);
                            hardware.slideDrive.setTargetPosition(ArmConfig.closeIntakeSlideCounts);
                        }
                    }
                }));

                //Slide Presets
                variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("a"), Gamepad.class.getField("y"), Boolean::equals, (Boolean a, Boolean y) -> {

                    if(a && y) {
                        return;
                    }

                    int pivotTarget = hardware.pivotDrive.getTargetPosition();
                    int slideTarget = hardware.slideDrive.getTargetPosition();

                    if(mode == Mode.NORMAL) {
                        if(y) {
                            if(slideTarget == ArmConfig.restingSlideCounts && pivotTarget == ArmConfig.restingPivotCounts) {
                                hardware.pivotDrive.setTargetPosition(ArmConfig.hoverPivotCounts);
                                hardware.slideDrive.setTargetPosition(ArmConfig.closeIntakeSlideCounts);

                            } else if(slideTarget == ArmConfig.restingSlideCounts && pivotTarget == ArmConfig.hoverPivotCounts) {
                                hardware.slideDrive.setTargetPosition(ArmConfig.closeIntakeSlideCounts);

                            } else if(slideTarget == ArmConfig.closeIntakeSlideCounts) {
                                hardware.pivotDrive.setTargetPosition(ArmConfig.hoverPivotCounts);
                                hardware.slideDrive.setTargetPosition(ArmConfig.farIntakeSlideCounts);

                            } else if (slideTarget == ArmConfig.restingSlideCounts && pivotTarget == ArmConfig.basketPivotCounts) {
                                hardware.slideDrive.setTargetPosition(ArmConfig.basketSlideCounts);
                            }
                        } else if(a) {
                            if(slideTarget == ArmConfig.farIntakeSlideCounts || slideTarget == ArmConfig.closeIntakeSlideCounts) {
                                hardware.pivotDrive.setTargetPosition(ArmConfig.hoverPivotCounts);
                            }

                            if(slideTarget == ArmConfig.basketSlideCounts) {
                                hardware.slideDrive.setTargetPosition(ArmConfig.restingSlideCounts);

                            } else if(slideTarget == ArmConfig.farIntakeSlideCounts) {
                                hardware.slideDrive.setTargetPosition(ArmConfig.closeIntakeSlideCounts);

                            } else if(slideTarget == ArmConfig.closeIntakeSlideCounts) {
                                hardware.slideDrive.setTargetPosition(ArmConfig.restingSlideCounts);
                            }
                        }
                    } else if(mode == Mode.SPECIMEN && pivotTarget == ArmConfig.specimenPivotCounts) {
                        if(a && slideTarget == ArmConfig.specimenApproachSlideCounts) {
                            hardware.slideDrive.setTargetPosition(ArmConfig.specimenAttachSlideCounts);
                        } else if(y && slideTarget == ArmConfig.specimenAttachSlideCounts) {
                            hardware.slideDrive.setTargetPosition(ArmConfig.specimenApproachSlideCounts);
                        }
                    }
                }));
                variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("x"), Boolean::equals, (Boolean x) -> {
                    if(x) {
                        if (mode == Mode.NORMAL && Math.abs(hardware.slideDrive.getTargetPosition()) <= Math.abs(ArmConfig.closeIntakeSlideCounts) && Math.abs(hardware.pivotDrive.getTargetPosition()) >= Math.abs(ArmConfig.closeIntakePivotCounts)) {
                            setMode(Mode.SPECIMEN);
                            hardware.slideDrive.setTargetPosition(ArmConfig.closeIntakeSlideCounts);
                            hardware.pivotDrive.setTargetPosition(ArmConfig.closeIntakePivotCounts);
                        } else if (mode == Mode.SPECIMEN) {
                            setMode(Mode.NORMAL);
                            hardware.slideDrive.setTargetPosition(ArmConfig.restingSlideCounts);
                            hardware.pivotDrive.setTargetPosition(ArmConfig.hoverPivotCounts);
                        }
                    }
                }));
                variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_trigger"), Gamepad.class.getField("right_trigger"), Float::equals, (Float left_trigger, Float right_trigger) -> {
                    Log.i("MainTeleop", "pivotmode: " + hardware.pivotDrive.getMode());
                    Log.i("MainTeleop", "pivottarget: " + hardware.pivotDrive.getTargetPosition());
                    Log.i("MainTeleop", "slidemode: " + hardware.slideDrive.getMode());
                    Log.i("MainTeleop", "slidetarget: " + hardware.slideDrive.getTargetPosition());
                    if(mode == Mode.HANG) {
                        double speed = 1000 * (left_trigger - right_trigger);
                        if(abs(speed) < 0.001) speed += 1;
                        hardware.slideDrive.setVelocity(speed * 2);
                    }
                }));
                variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("dpad_left"), Gamepad.class.getField("b"), Boolean::equals, (Boolean dpad_left, Boolean b) -> {
                    if(dpad_left && b) {
                        if(mode == Mode.NORMAL || mode == Mode.SPECIMEN) {
                            setMode(Mode.HANG);
                            hardware.pivotDrive.setTargetPosition(ArmConfig.hangPivotCounts);
                            hardware.slideDrive.setPower(ArmConfig.slidePower);
                            hardware.slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            hardware.slideDrive.setVelocity(1);
                        } else if(mode == Mode.HANG) {
                            setMode(Mode.NORMAL);
                            hardware.slideDrive.setPower(ArmConfig.slidePower);
                            hardware.slideDrive.setVelocity(0);
                            hardware.slideDrive.setTargetPosition(ArmConfig.restingSlideCounts);
                            hardware.slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            hardware.pivotDrive.setTargetPosition(ArmConfig.hoverPivotCounts);

                        }
                    }
                }));
                variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("right_stick_button"), Boolean::equals, (Boolean right_stick_button) -> {
                    if(right_stick_button) {
                        if(mode == Mode.NORMAL || mode == Mode.HOMING) {
                            startHoming(HomingDirection.OUT);
                        }  else if(mode == Mode.HOMING_OUT) {
                            setMode(Mode.NORMAL);
                            hardware.slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            MainBotHardware.runMotorToPos(hardware.slideDrive, ArmConfig.restingSlideCounts, ArmConfig.slidePower);
                        }
                    }
                }));
                //Run grabber
                variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_bumper"), Gamepad.class.getField("right_bumper"), Boolean::equals, (Boolean left_bumper, Boolean right_bumper) -> {
                    if(mode != Mode.NORMAL && mode != Mode.SPECIMEN) {
                        return;
                    }

                    if(right_bumper == left_bumper) {
                        hardware.intakeServo.setPosition(ArmConfig.restingServoPos);
                        hardware.intakeServo2.setPosition(1-ArmConfig.restingServoPos);
                    } else if (right_bumper) {
                        double servoPos;
                        if(hardware.pivotDrive.getTargetPosition() == ArmConfig.basketPivotCounts && hardware.slideDrive.getTargetPosition() == ArmConfig.basketSlideCounts) {
                            servoPos = ArmConfig.dropServoPos;
                        } else {
                            servoPos = ArmConfig.expelServoPos;
                        }
                        hardware.intakeServo.setPosition(servoPos);
                        hardware.intakeServo2.setPosition(1 - servoPos);
                    } else if (left_bumper) {
                        hardware.intakeServo.setPosition(ArmConfig.grabServoPos);
                        hardware.intakeServo2.setPosition(1-ArmConfig.grabServoPos);
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
                    if(haveIncorrectColor && warningOn) {
                        hardware.blinkin.setPattern(mode.toColor());
                        warningOn = false;
                    }
                    haveIncorrectColor = false;
                }

//                int currentColor = hardware.colorSensor.argb();
//                float[] hsv = new float[3];
//                Color.colorToHSV(hardware.colorSensor.argb(), hsv);
//                float hue = hsv[0];
//
//                if(
//                    Color.alpha(currentColor) > sampleDetectedAlpha &&
//                    ((hue > blueMinSaturation && hue < blueMaxSaturation && alliance == Alliance.RED) ||
//                    (hue > redMinSaturation && hue < redMaxSaturation && alliance == Alliance.BLUE))
//                ) {
//                    haveIncorrectColor = true;
//                } else {
//                    if(haveIncorrectColor && warningOn) {
//                        hardware.blinkin.setPattern(mode.toColor());
//                        warningOn = false;
//                    }
//                    haveIncorrectColor = false;
//                }
            }));

            timedEvents.add(new TimedEvent(incorrectColorFlashTime, () -> {
                if(haveIncorrectColor) {
                    if(warningOn) {
                        hardware.blinkin.setPattern(mode.toColor());
                    } else {
                        hardware.blinkin.setPattern(wrongSample);
                    }
                    warningOn = !warningOn;
                }
            }));
        }
        startHoming(HomingDirection.IN);
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

