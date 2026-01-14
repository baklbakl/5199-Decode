package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.TimedEvent;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.TripleVariableWatcher;
import org.ducktapeandaprayer.ftc.vision.AprilTagPoseEx;

import static java.lang.Math.abs;
import static org.ducktapeandaprayer.ftc.auto.roadrunner.ThreeDeadWheelLocalizer.PARAMS;

public class ControlHubThread extends ThreadEx {
    //
    // Dash Options
    //

    public static double autoAimKS = 0.1;
    public static double autoAimKP = 0.1;
    public static double autoAimCutoffAngle = 1;

    //    public final AtomicBoolean autoAim = new AtomicBoolean(false);

    //
    // Global Variables
    //

    private final MainBotHardware hardware;
    private final ThreadedTele threadedTele;

    private double autoAimTurningPower = 0;
    private double autoAimTargetHeading = 0;
    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastGamePadTurn = 0;
    private boolean autoAim = false;
    private int startingEncoder0Ticks = 0;
    private int startingEncoder1Ticks = 0;

    private void computeWheelSpeeds(double forward, double strafe, double turning) {

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

        if(autoAim) {
            throw new RuntimeException("Can't start auto aiming while auto aiming is already started");
        }

        startingEncoder0Ticks = hardware.parallelEncoder0.getPositionAndVelocity().position;
        startingEncoder1Ticks = hardware.parallelEncoder1.getPositionAndVelocity().position;
        computeTargetHeading();
        this.autoAim = true;
    }

    private void stopAutoAim() {
        if(!autoAim) {
            throw new RuntimeException("Can't stop auto aiming while auto aiming is already stopped");
        }

        computeWheelSpeeds(lastForward, lastStrafe, lastGamePadTurn);
        this.autoAim = false;
    }

    private double computeHeading() {
        double currEncoder0Ticks = startingEncoder0Ticks - hardware.parallelEncoder0.getPositionAndVelocity().position;
        double currEncoder1Ticks = startingEncoder1Ticks - hardware.parallelEncoder1.getPositionAndVelocity().position;
        return ((currEncoder0Ticks - currEncoder1Ticks) / (PARAMS.par0YTicks - PARAMS.par1YTicks)) % (Math.PI * 2);
    }

    private void computeTargetHeading() {
        //Positive x is to the right
        double x;
        double groundDistance;

        synchronized (threadedTele.tagPoseLock) {
            AprilTagPoseEx pose = threadedTele.getTagPose();
            x = pose.x;
            groundDistance = pose.groundDistance;
        }

        double angularError = Math.atan2(x, groundDistance);

        //Subtract because positive x is to the right
        autoAimTargetHeading = computeHeading() - angularError;
    }

    public ControlHubThread(MainBotHardware hardwareArg, ThreadedTele threadedTele, Gamepad gamepad1) {
        this.hardware = hardwareArg;
        this.threadedTele = threadedTele;
        hardware.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        try {
            //Movement
            super.variablesWatched.add(new TripleVariableWatcher<>(gamepad1, Gamepad.class.getDeclaredField("left_stick_x"), Gamepad.class.getDeclaredField("left_stick_y"), Gamepad.class.getDeclaredField("right_stick_x"), Float::equals,
                    (Float left_stick_x_f, Float left_stick_y_f, Float right_stick_x) -> {
                        lastGamePadTurn = right_stick_x;

                        // Note: pushing stick forward gives negative value
                        double turning;
                        if(autoAim) {
                            turning = autoAimTurningPower;
                        } else {
                            turning = lastGamePadTurn;
                        }

                        lastForward = -left_stick_y_f;
                        lastStrafe = left_stick_x_f;

                        computeWheelSpeeds(lastForward, lastStrafe, turning);
                    }
            ));

            super.variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("square"), Boolean::equals, (Boolean square) -> {
                if(!square) {
                    return;
                }

                if(!threadedTele.runVision) {
                    return;
                }

                if(autoAim) {
                    stopAutoAim();
                } else {
                    startAutoAim();
                }
            }));


        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        this.timedEvents.add(new TimedEvent(30, this::computeTargetHeading));
    }

    public void tick() {
        if(autoAim) {
            double headingError = autoAimTargetHeading - computeHeading();
            if(headingError > Math.toRadians(autoAimCutoffAngle)) {
                autoAimTurningPower = Math.signum(headingError) * autoAimKS + headingError * autoAimKP;
                computeWheelSpeeds(lastForward, lastStrafe, autoAimTurningPower);
            }
        }

        updateTelemetry();
        hardware.modules.parentModule.module.clearBulkCache();
    }
}
