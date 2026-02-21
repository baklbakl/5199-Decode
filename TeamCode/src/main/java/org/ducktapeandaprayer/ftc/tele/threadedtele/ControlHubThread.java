package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.TimedEvent;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.TripleVariableWatcher;
import org.ducktapeandaprayer.ftc.vision.AprilTagPoseEx;

import java.util.concurrent.atomic.AtomicReference;

import static java.lang.Math.abs;
import static org.ducktapeandaprayer.ftc.auto.roadrunner.ThreeDeadWheelLocalizer.PARAMS;

public class ControlHubThread extends ThreadEx {
    //
    // Dash Options
    //

    public static double autoAimKS = -0.08;
    public static double autoAimKP = -0.6;
    public static double autoAimCutoffAngle = 1;
    public static double autoAimThreshAngle = 3;
    public static double autoAimXOffsetIn = 5;

    //
    // Global Variables
    //

    public Boolean autoAimDebugInfoEnable = true;
    public Boolean autoAimOptionDriverEnable = true;

    private final MainBotHardware hardware;
    private final ThreadedTele threadedTele;

    private double autoAimTurningPower = 0;
    private double autoAimTargetHeading = 0;
    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastGamePadTurn = 0;

    private volatile AutoAimState autoAimState = AutoAimState.DISABLED;
    private final AtomicReference<Boolean> requestAutoAimStatus = new AtomicReference<>(null);

    private int startingEncoder0Ticks = 0;
    private int startingEncoder1Ticks = 0;

    public enum AutoAimState {
        //Disabled means that we auto aim will never start and the system is disabled
        DISABLED,
        //Stopped means the system is running, but not currently controlling the robot heading
        STOPPED,
        NO_TAG_FOUND,
        LOCKING,
        LOCKED
    }

    public AutoAimState getAutoAimState() {
        return autoAimState;
    }

    public void requestAutoAimStartStop(boolean running) {
        if(!running) {
            RobotLog.logStackTrace(Thread.currentThread(), "Control Hub Thread stop auto aim");
        }

        requestAutoAimStatus.set(running);
    }

    private void computeSetWheelSpeeds(double forward, double strafe, double turning) {

        double leftFrontPower  = forward + strafe + turning;
        double rightFrontPower = forward - strafe - turning;
        double leftBackPower   = forward - strafe + turning;
        double rightBackPower  = forward + strafe - turning;

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

    private void startAutoAim() {
        if(!threadedTele.runVision) {
            throw new RuntimeException("Can't start auto aiming without vision running");
        }

        if(autoAimState == AutoAimState.DISABLED) {
            throw new RuntimeException("Can't start auto aiming while auto aiming is disabled");
        }

        if(autoAimState != AutoAimState.STOPPED) {
            throw new RuntimeException("Can't start auto aiming while auto aiming is already started");
        }

        startingEncoder0Ticks = hardware.parallelEncoder0.getPositionAndVelocity().position;
        startingEncoder1Ticks = hardware.parallelEncoder1.getPositionAndVelocity().position;

        //FIX: This may take a cycle to update into locked if we started locked
        autoAimState = AutoAimState.LOCKING;
        computeTargetHeading();
    }

    private void stopAutoAim() {
        if(autoAimState == AutoAimState.DISABLED) {
            throw new RuntimeException("Can't stop auto aiming while auto aiming is disabled");
        }

        if(autoAimState == AutoAimState.STOPPED) {
            throw new RuntimeException("Can't stop auto aiming while auto aiming is already stopped");
        }

        computeSetWheelSpeeds(lastForward, lastStrafe, lastGamePadTurn);
        autoAimState = AutoAimState.STOPPED;
    }

    private String autoAimDebugInfo = "";

    private double computeHeading() {
        double currEncoder0Ticks = startingEncoder0Ticks - hardware.parallelEncoder0.getPositionAndVelocity().position;
        double currEncoder1Ticks = startingEncoder1Ticks - hardware.parallelEncoder1.getPositionAndVelocity().position;
        return ((currEncoder0Ticks - currEncoder1Ticks) / (PARAMS.par0YTicks - PARAMS.par1YTicks)) % (Math.PI * 2);
    }

    private void computeTargetHeading() {
        //Positive x is to the right
        double x = Double.NaN;
        double groundDistance = Double.NaN;
        boolean poseNull;

        synchronized (threadedTele.tagPoseLock) {
            AprilTagPoseEx pose = threadedTele.getTagPose();
            poseNull = pose == null;
            if(!poseNull) {
                x = pose.x;
                groundDistance = pose.groundDistance;
            }
        }

        StringBuilder sb = new StringBuilder();

        double realHeading = computeHeading();
        sb.append("\nOdometry Heading: ");
        sb.append(Math.toDegrees(realHeading));

        if(poseNull) {
            autoAimTargetHeading = Double.NaN;
        } else {
            sb.append("\nx: ");
            sb.append(x);

            x = x + autoAimXOffsetIn;
            sb.append("\ncorrected x: ");
            sb.append(x);

            double angularError = Math.atan2(x, groundDistance);
            sb.append("\nVision angular error: ");
            sb.append(Math.toDegrees(angularError));

            //Subtract because positive x is to the right
            autoAimTargetHeading = realHeading - angularError;
        }

        sb.append("\nAuto Aim Target Heading: ");
        sb.append(Math.toDegrees(autoAimTargetHeading));
        sb.append("\n");

        autoAimDebugInfo = sb.toString();
    }

    public ControlHubThread(MainBotHardware hardwareArg, ThreadedTele threadedTele, Gamepad gamepad1, OptionDriver optionDriver) {
        this.hardware = hardwareArg;
        this.threadedTele = threadedTele;
        hardware.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        try {
            optionDriver.add(new BooleanOption("Auto Aim debug info", "dpad_left", gamepad1, this.getClass().getField("autoAimDebugInfoEnable"), this));
            optionDriver.add(new BooleanOption("Run Auto Aim", "square", gamepad1, this.getClass().getField("autoAimOptionDriverEnable"), this));

            //Movement
            super.variablesWatched.add(new TripleVariableWatcher<>(gamepad1, Gamepad.class.getDeclaredField("left_stick_x"), Gamepad.class.getDeclaredField("left_stick_y"), Gamepad.class.getDeclaredField("right_stick_x"), Float::equals,
                    (Float left_stick_x_f, Float left_stick_y_f, Float right_stick_x) -> {
                        lastGamePadTurn = right_stick_x;

                        // Note: pushing stick forward gives negative value
                        double turning;
                        if(autoAimState == AutoAimState.LOCKING || autoAimState == AutoAimState.LOCKED) {
                            turning = autoAimTurningPower;
                        } else {
                            turning = lastGamePadTurn;
                        }


                        lastForward = -left_stick_y_f;
                        lastStrafe = left_stick_x_f;

                        computeSetWheelSpeeds(lastForward, lastStrafe, turning);
                    }
            ));

            //This is inherently somewhat unsafe and only should be ran if the robot is at idle
            super.variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("share"), Boolean::equals, (Boolean share) -> {
                if(!share) {
                    return;
                }

                if(!threadedTele.runVision) {
                    return;
                }

                if(autoAimState == AutoAimState.DISABLED) {
                   autoAimState = AutoAimState.STOPPED;
                } else {
                    if(autoAimState != AutoAimState.STOPPED) {
                        stopAutoAim();
                    }

                    autoAimState = AutoAimState.DISABLED;
                }
            }));

            super.variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("square"), Boolean::equals, (Boolean square) -> {
                if(!square) {
                    return;
                }

                if(!threadedTele.runVision) {
                    return;
                }

                if(autoAimState == AutoAimState.DISABLED) {
                    return;
                }

                if(autoAimState == AutoAimState.STOPPED) {
                    startAutoAim();
                } else {
                    //The user can stop auto aim during auto launch, but that's okay they may have a valid reason to cancel.
                    stopAutoAim();
                }
            }));


        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        this.timedEvents.add(new TimedEvent(30, () -> {
            if(threadedTele.runVision) {
                computeTargetHeading();
            }
        }));
    }

    @Override
    public void run() {
        if(autoAimOptionDriverEnable && threadedTele.runVision) {
            autoAimState = AutoAimState.STOPPED;
        } else {
            autoAimDebugInfoEnable = false;
            autoAimState = AutoAimState.DISABLED;
        }

        super.run();
    }

    public void tick() {
        telemetry.append("Auto Aim State: ");
        telemetry.append(autoAimState);
        telemetry.append("\n");

        if((autoAimState != AutoAimState.DISABLED && autoAimState != AutoAimState.STOPPED)) {
            if(Double.isNaN(autoAimTargetHeading)) {
                if(autoAimState != AutoAimState.NO_TAG_FOUND) {
                    autoAimState = AutoAimState.NO_TAG_FOUND;
                    autoAimTurningPower = 0;
                    computeSetWheelSpeeds(lastForward, lastStrafe, lastGamePadTurn);
                }
            } else if(autoAimState == AutoAimState.NO_TAG_FOUND) {
                autoAimState = AutoAimState.LOCKING;
            }
        }

        if(autoAimState == AutoAimState.LOCKED || autoAimState == AutoAimState.LOCKING) {
            double headingError = autoAimTargetHeading - computeHeading();
            if(Math.abs(headingError) > Math.toRadians(autoAimCutoffAngle)) {
                autoAimTurningPower = Math.signum(headingError) * autoAimKS + headingError * autoAimKP;
            } else {
                autoAimTurningPower = 0;
            }

            computeSetWheelSpeeds(lastForward, lastStrafe, autoAimTurningPower);

//            autoAimState = Math.abs(headingError) <                                                                                                                                                                            th.toRadians(autoAimThreshAngle) ? AutoAimState.LOCKED : AutoAimState.LOCKING;
            autoAimState = Math.abs(headingError) < Math.toRadians(autoAimThreshAngle) ? AutoAimState.LOCKED : AutoAimState.LOCKING;
        }

        if(autoAimDebugInfoEnable) {
            telemetry.append("Turning Power: ");
            telemetry.append(autoAimTurningPower);
            telemetry.append("\n");
        }

        if(autoAimState != AutoAimState.DISABLED) {
            if(autoAimDebugInfoEnable) {
                telemetry.append(autoAimDebugInfo);
            }

            Boolean requestAutoAimLocal;
            synchronized (requestAutoAimStatus) {
                requestAutoAimLocal = requestAutoAimStatus.get();
                if(requestAutoAimLocal != null) {
                    requestAutoAimStatus.set(null);
                }
            }

            if(requestAutoAimLocal != null) {
                if(requestAutoAimLocal) {
                    startAutoAim();
                } else {
                    stopAutoAim();
                }
            }
        }

        updateTelemetry();
        hardware.modules.parentModule.module.clearBulkCache();
    }
}
