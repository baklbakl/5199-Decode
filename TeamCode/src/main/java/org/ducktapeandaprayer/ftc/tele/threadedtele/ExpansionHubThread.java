package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.apache.commons.lang3.tuple.Pair;
import org.ducktapeandaprayer.ftc.hardware.BeamBreak;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.TwoOfThree;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.TripleVariableWatcher;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

public class ExpansionHubThread extends ThreadEx {
    //
    // Dash options
    //
    //FIX: Figure out the correct turret offset
    public static double turretOffset = -52.0 /*mm*/ / 25.4;
    public static Vector2d goalTargetPos = new Vector2d(-66, 66);

    //
    // Globals
    //

    public Boolean autoLockDebInfo = true;

    private final MainBotHardware hardware;
    private final Gamepad gamepad1;
    private final ThreadedTele threadedTele;
    private final ControlHubThread controlHubThread;

    private Vector2d correctedGoalTargetPose;

    private enum LauncherControlMethod {
        MANUAL,
        PRESET,
        ODOMETRY,
    }
    private final List<LauncherControlMethod> launcherControlMethodElements = new ArrayList<>();
    private LauncherControlMethod launcherControlMethod;
    private boolean turretTracking = true;
    private final ElapsedTime timer = new ElapsedTime();
    private double shootingStartTimeForTransfer = Double.NaN;
    private double shootingStartTimeForRamp = Double.NaN;

    private enum State {
        STOPPED,
        INTAKE,
        OUTTAKE,
        SHOOTING,
    }

    private void toState(State state) {
        this.state = state;
        switch (state) {
            case STOPPED:
                hardware.transferDrive.setPower(0);
                hardware.intakeDrive.setPower(0);
                hardware.rampServo.setPosition(MainBotHardware.rampDownPos);
                break;
            case INTAKE:
                hardware.transferDrive.setPower(MainBotHardware.transferInPower);
                hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
                hardware.rampServo.setPosition(MainBotHardware.rampDownPos);
                break;
            case OUTTAKE:
                hardware.transferDrive.setPower(-MainBotHardware.transferInPower);
                hardware.intakeDrive.setPower(-MainBotHardware.intakeInPower);
                hardware.rampServo.setPosition(MainBotHardware.rampDownPos);
                break;
            case SHOOTING:
                //hardware.transferDrive.setPower(MainBotHardware.transferInPower);
                //hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
                shootingStartTimeForTransfer = timer.time();
                shootingStartTimeForRamp = shootingStartTimeForTransfer;
                hardware.rampServo.setPosition(MainBotHardware.rampShootPos);
                break;
        }
    }

    private State state = State.STOPPED;

    private static double launchSpeedFromDist(double dist) {
        synchronized (MainBotHardware.launchSpeedMapping) {
            Pair<Double, Double> lowestPair = MainBotHardware.launchSpeedMapping.get(0);
            if(dist < lowestPair.getKey()) {
                //return Double.NaN;
                return lowestPair.getValue();
            }


            for(int i = 0; i < MainBotHardware.launchSpeedMapping.size(); i++) {
                Pair<Double, Double> p = MainBotHardware.launchSpeedMapping.get(i);
                if(dist == p.getKey()) {
                    return p.getValue();
                }

                if(dist < p.getKey()) {
                    //Previous one must have been greater, so we know we are between the two closest points,
                    // so we interpolate with a weighted average
                    Pair<Double, Double> last = MainBotHardware.launchSpeedMapping.get(i-1);
                    double deltaDist = p.getKey() - last.getKey();
                    double offset = dist - last.getKey();
                    double weightLast = (deltaDist - offset) / deltaDist;
                    double weightThis = offset / deltaDist;

                    return weightLast * last.getValue() + weightThis * p.getValue();
                }
            }
        }

        //return Double.NaN;
        return MainBotHardware.launchSpeedMapping.get(MainBotHardware.launchSpeedMapping.size() - 1).getValue();
    }

    private boolean startOdoLauncher = false;

    private void startLauncher() {
        if(launcherControlMethod == LauncherControlMethod.PRESET) {
            hardware.launchDrive.setVelocity(MainBotHardware.launcherPresetVelocity);
        } else if(launcherControlMethod == LauncherControlMethod.ODOMETRY) {
            //hardware.launchDrive.setVelocity(computeOdoLauncherSpeed());
            //This will get automatically set in the tick method
            startOdoLauncher = true;
        } else {
            throw new RuntimeException("Oops unaccounted for launcher control method \"" + launcherControlMethod + "\"");
        }
    }

    public ExpansionHubThread(MainBotHardware hardwareArg, ThreadedTele threadedTele, ControlHubThread controlHubThread, Gamepad gamepad1, OptionDriver optionDriver) {
        this.hardware = hardwareArg;
        this.threadedTele = threadedTele;
        this.controlHubThread = controlHubThread;
        this.gamepad1 = gamepad1;

        hardware.modules.childModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        launcherControlMethodElements.addAll(EnumSet.allOf(LauncherControlMethod.class));
        launcherControlMethod = LauncherControlMethod.ODOMETRY;

        try {
            optionDriver.add(new BooleanOption("Auto lock debug info", "dpad_right", gamepad1, this.getClass().getField("autoLockDebInfo"), this));

            variablesWatched.add(new TripleVariableWatcher<>(
                    gamepad1,
                    Gamepad.class.getField("left_bumper"),
                    Gamepad.class.getField("square"),
                    Gamepad.class.getField("right_bumper"),
                    Boolean::equals, (Boolean left_bumper, Boolean square, Boolean right_bumper) -> {
                if(TwoOfThree.twoOfThree(left_bumper, square, right_bumper) || (!left_bumper && !square && !right_bumper)) {
                    toState(State.STOPPED);
                } else if(square) {
                    toState(State.OUTTAKE);
                } else if(left_bumper) {
                    toState(State.INTAKE);
                } else if(right_bumper) {
                    toState(State.SHOOTING);
                } else {
                    throw new RuntimeException("Oopsy. Something bad happened. Never should be here");
                }
            }));


            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("triangle"), (Boolean triangle) -> {
                if(triangle && launcherControlMethod != LauncherControlMethod.MANUAL) {
                    if(hardware.launchDrive.getTargetVelocity() == 0) {
                        startLauncher();
                    } else {
                        hardware.launchDrive.setVelocity(0);
                    }
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("circle"), (Boolean circle) -> {
                if(circle) {
                    turretTracking = !turretTracking;
                    if(!turretTracking) {
                        hardware.turretToHeading(0);
                    }
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_trigger"), Gamepad.class.getField("right_trigger"), Float::equals, (Float left_trigger, Float right_trigger) -> {
                if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                    float power = left_trigger - right_trigger;
                    hardware.launchDrive.setPower(power);
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_stick_button"), Gamepad.class.getField("right_stick_button"), Boolean::equals, (Boolean left_stick_button, Boolean right_stick_button) -> {
                if(left_stick_button && right_stick_button) {
                    int state = launcherControlMethodElements.indexOf(launcherControlMethod) + 1;
                    state %= launcherControlMethodElements.size();
                    launcherControlMethod = launcherControlMethodElements.get(state);

                    if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hardware.launchDrive.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                    } else {
                        hardware.launchDrive.setVelocity(0);
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }
            }));

        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void run() {
        if(threadedTele.getAlliance() == Alliance.BLUE) {
            correctedGoalTargetPose = new Vector2d(goalTargetPos.x, -goalTargetPos.y);
        } else if(threadedTele.getAlliance() == Alliance.RED) {
            correctedGoalTargetPose = goalTargetPos;
        }

        timer.reset();
        startOdoLauncher = true;

        super.run();
    }

    public void tick() {
        hardware.modules.childModule.module.clearBulkCache();
        hardware.tickBBs();

        Pose2d robotPose = controlHubThread.robotPose.get();
        Pose2d turretPose = robotPose.plus(new Twist2d(new Vector2d(turretOffset, 0), 0));
        double distToGoal = correctedGoalTargetPose.minus(turretPose.position).norm();
        double autoSpeed = launchSpeedFromDist(distToGoal);

        if(autoLockDebInfo) {
            telemetry.append("Dist to goal: ");
            telemetry.append(distToGoal);
            telemetry.append("\n");
            telemetry.append("Launcher Auto Speed Target: ");
            telemetry.append(autoSpeed);
            telemetry.append("\n");
        }

        if(launcherControlMethod == LauncherControlMethod.ODOMETRY && (hardware.launchDrive.getTargetVelocity() != 0) || startOdoLauncher) {
            startOdoLauncher = false;
            hardware.launchDrive.setVelocity(autoSpeed);
        }

        double fieldThetaToGoal = Math.atan2(correctedGoalTargetPose.y - turretPose.position.y, correctedGoalTargetPose.x - turretPose.position.x);
        double turretTheta = (fieldThetaToGoal - turretPose.heading.toDouble() + Math.PI * 2) % (Math.PI * 2);
        if(turretTheta > Math.PI) {
            turretTheta -= 2 * Math.PI;
        }
        if(autoLockDebInfo) {
            telemetry.append("Turret tracking: ");
            telemetry.append(turretTracking);
            telemetry.append("\n");
            telemetry.append("Field angle to goal: ");
            telemetry.append(Math.toDegrees(fieldThetaToGoal));
            telemetry.append("\n");
            telemetry.append("Turret target theta: ");
            telemetry.append(Math.toDegrees(turretTheta));
            telemetry.append("\n");
        }

        if(turretTracking) {
            hardware.turretToHeading(turretTheta);
        }

        switch(state) {
            case STOPPED:
            case OUTTAKE:
                break;

            case INTAKE:
                //If transfer full stop it. If robot full stop intaking and rumble controller
                if(hardware.transferMiddleBB.getState() == BeamBreak.State.FULL && hardware.transferEndBB.getState() == BeamBreak.State.FULL) {
                    hardware.transferDrive.setPower(0);

                    if(hardware.intakeBB.getState() == BeamBreak.State.FULL) {
                        hardware.intakeDrive.setPower(0);
                        gamepad1.rumble(200);
                        state = State.STOPPED;
                    }
                }

                break;

            case SHOOTING:
                //Shooting start time is Double.NaN when not waiting to start the transfer and 1 - NaN = NaN and NaN >= any number = false, so this works
                double time = timer.seconds();
                if(time - shootingStartTimeForTransfer >= MainBotHardware.shootTransferOnDelay) {
                    hardware.transferDrive.setPower(MainBotHardware.transferInPower);
                    hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
                    shootingStartTimeForTransfer = Double.NaN;
                }

                if(hardware.intakeBB.getState() == BeamBreak.State.EMPTY && hardware.transferMiddleBB.getState() == BeamBreak.State.EMPTY && time - shootingStartTimeForRamp >= MainBotHardware.singleShootHighRampDelay) {
                    hardware.rampServo.setPosition(MainBotHardware.rampLastBallPos);
                    shootingStartTimeForRamp = Double.NaN;
                }
                break;
        }

        telemetry.append("\nState: ");
        telemetry.append(state);
        telemetry.append("\nIntake BB: ");
        telemetry.append(hardware.intakeBB.getState());
        telemetry.append("\nTransfer Middle BB: ");
        telemetry.append(hardware.transferMiddleBB.getState());
        telemetry.append("\nTransfer End BB: ");
        telemetry.append(hardware.transferEndBB.getState());
        telemetry.append("\n\n");

        hardware.voltageSensor.periodic();

        telemetry.append("Launcher Target Speed: ");
        telemetry.append(hardware.launchDrive.getTargetVelocity());
        telemetry.append("\nLauncher Current Speed: ");
        telemetry.append(hardware.launchDrive.getVelocity());
        telemetry.append("\nLauncher Control Method: ");
        telemetry.append(launcherControlMethod);
        telemetry.append("\n\n");

        updateTelemetry();
    }
}
