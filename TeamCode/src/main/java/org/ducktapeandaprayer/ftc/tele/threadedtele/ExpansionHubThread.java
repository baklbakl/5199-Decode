package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.apache.commons.lang3.tuple.Pair;
import org.ducktapeandaprayer.ftc.hardware.*;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.TwoOfThree;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.TripleVariableWatcher;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import static org.ducktapeandaprayer.ftc.lib.Alliance.RED;

public class ExpansionHubThread extends ThreadEx {
    //
    // Dash options
    //
    public static class AutoGateCyclePosePair {
        public final Pose2d shootPose;
        public final Pose2d gateLineupPose;
        public AutoGateCyclePosePair(Pose2d shootPose, Pose2d gateLineupPose) {
            this.shootPose = shootPose;
            this.gateLineupPose = gateLineupPose;
        }
    }
    public static final AutoGateCyclePosePair nearAutoGateCyclePose = new AutoGateCyclePosePair(
            new Pose2d(-18, 25, Math.PI / 4),
            new Pose2d(5, 39, Math.PI / 4)
    );

    public static final AutoGateCyclePosePair midAutoGateCyclePose = new AutoGateCyclePosePair(
            new Pose2d(-8, 15, Math.PI / 3),
            new Pose2d(5, 39, Math.PI / 3)
    );

    public static final AutoGateCyclePosePair farAutoGateCyclePose = new AutoGateCyclePosePair(
            new Pose2d(2, 5, Math.PI / 2),
            null
    );

    public static Pose2d gatePose = new Pose2d(9, 47, Math.PI / 2);

    //
    // Globals
    //

    public Boolean autoRangingDebugInfo = true;

    private final MainBotHardware hardware;
    private MainBotHardware.Container container;
    private final Gamepad gamepad1;
    private final ThreadedTele threadedTele;
    private final ControlHubThread controlHubThread;
    private final AutoShoot autoShoot;
    private final AutoIntake autoIntake;
    private final AutoRanging autoRanging;

    private enum LauncherControlMethod {
        MANUAL,
        PRESET,
        ODOMETRY,
    }
    private final List<LauncherControlMethod> launcherControlMethodElements = new ArrayList<>();
    private LauncherControlMethod launcherControlMethod;
    private enum AutoGateCycleState {
        DISABLED,
        TO_SHOOT_POS,
        SHOOTING,
        TO_GATE
    }
    private AutoGateCycleState autoGateCycleState = AutoGateCycleState.DISABLED;

    private enum BallHandlingState {
        STOPPED,
        INTAKE,
        OUTTAKE,
        SHOOTING,
    }

    private void toBallHandlingState(BallHandlingState newState) {
        if(newState == ballHandlingState) return;
        switch (newState) {
            case STOPPED:
                container.restIntakeTransfer();
                break;
            case INTAKE:
                autoIntake.startIntaking();
                break;
            case OUTTAKE:
                container.transferDrive.setPower(-MainBotHardware.transferInPower);
                container.intakeDrive.setPower(-MainBotHardware.intakeInPower);
                container.rampServo.setPosition(MainBotHardware.rampDownPos);
                break;
            case SHOOTING:
                autoShoot.startShooting();
                break;
        }
        this.ballHandlingState = newState;
    }
    private BallHandlingState ballHandlingState = BallHandlingState.STOPPED;

    private AutoGateCyclePosePair activeAutoGateCyclePosePair = null;

    private void startLauncher() {
        if(launcherControlMethod == LauncherControlMethod.PRESET) {
            container.launchDrive.setVelocity(MainBotHardware.launcherPresetVelocity);
        } else if(launcherControlMethod == LauncherControlMethod.ODOMETRY) {
            //This will get automatically set in the tick method
            autoRanging.setLauncherRunning(true);
        } else {
            throw new RuntimeException("Oops unaccounted for launcher control method \"" + launcherControlMethod + "\"");
        }
    }

    public ExpansionHubThread(MainBotHardware hardwareArg, ThreadedTele threadedTele, ControlHubThread controlHubThread) {
        this.hardware = hardwareArg;
        this.threadedTele = threadedTele;
        this.controlHubThread = controlHubThread;
        this.gamepad1 = threadedTele.gamepad1;
        this.autoShoot = new AutoShoot(hardware);
        this.autoIntake = new AutoIntake(hardware);
        this.autoRanging = new AutoRanging(hardware, threadedTele::invertIfBlue);

        launcherControlMethodElements.addAll(EnumSet.allOf(LauncherControlMethod.class));
        launcherControlMethod = LauncherControlMethod.ODOMETRY;

        OptionDriver optionDriver = threadedTele.getOptionDriver();

        try {
            optionDriver.add(new BooleanOption("Auto ranging debug info", "dpad_right", gamepad1, this.getClass().getField("autoRangingDebugInfo"), this));

            variablesWatched.add(new TripleVariableWatcher<>(
                    gamepad1,
                    Gamepad.class.getField("left_bumper"),
                    Gamepad.class.getField("square"),
                    Gamepad.class.getField("right_bumper"),
                    Boolean::equals, (Boolean left_bumper, Boolean square, Boolean right_bumper) -> {
                if(autoGateCycleState != AutoGateCycleState.DISABLED) {
                    return;
                }

                if(TwoOfThree.twoOfThree(left_bumper, square, right_bumper) || (!left_bumper && !square && !right_bumper)) {
                    toBallHandlingState(BallHandlingState.STOPPED);
                } else if(square) {
                    toBallHandlingState(BallHandlingState.OUTTAKE);
                } else if(left_bumper) {
                    toBallHandlingState(BallHandlingState.INTAKE);
                } else if(right_bumper) {
                    toBallHandlingState(BallHandlingState.SHOOTING);
                } else {
                    throw new RuntimeException("Oopsy. Something bad happened. Never should be here");
                }
            }));

            variablesWatched.add(new TripleVariableWatcher<>(gamepad1, Gamepad.class.getField("dpad_left"), Gamepad.class.getField("dpad_down"), Gamepad.class.getField("dpad_right"), (Boolean dpad_left, Boolean dpad_down, Boolean dpad_right) -> {
                if(TwoOfThree.twoOfThree(dpad_left, dpad_down, dpad_right)) {
                    return;
                }

                if(dpad_left || dpad_down || dpad_right) {
                    Alliance alliance = threadedTele.getAlliance();
                    if(dpad_left && alliance == RED || dpad_right && alliance == Alliance.BLUE) {
                        activeAutoGateCyclePosePair = nearAutoGateCyclePose;
                    } else if(dpad_down) {
                        activeAutoGateCyclePosePair = midAutoGateCyclePose;
                    } else if (dpad_right && alliance == RED || dpad_left && alliance == Alliance.BLUE) {
                        activeAutoGateCyclePosePair = farAutoGateCyclePose;
                    }

                    if(container.transferEndBB.getState() == BeamBreak.State.EMPTY && container.transferMiddleBB.getState() == BeamBreak.State.EMPTY && container.intakeBB.getState() == BeamBreak.State.EMPTY) {
                        return;
                    }
                    toBallHandlingState(BallHandlingState.STOPPED);
                    autoGateCycleState = AutoGateCycleState.TO_SHOOT_POS;
                    controlHubThread.requestFollowerFollow(List.of(activeAutoGateCyclePosePair.shootPose));

                } else {
                    if(autoGateCycleState != AutoGateCycleState.DISABLED) {
                        autoGateCycleState = AutoGateCycleState.DISABLED;
                        controlHubThread.requestFollowerCancel();
                    }

                    activeAutoGateCyclePosePair = null;

                    if(gamepad1.left_bumper) {
                        toBallHandlingState(BallHandlingState.INTAKE);
                    } else {
                        toBallHandlingState(BallHandlingState.STOPPED);
                    }
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("triangle"), (Boolean triangle) -> {
                if(triangle && launcherControlMethod != LauncherControlMethod.MANUAL) {
                    if(container.launchDrive.getTargetVelocity() == 0) {
                        startLauncher();
                    } else {
                        container.launchDrive.setVelocity(0);
                    }
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("circle"), (Boolean circle) -> {
                if(circle) {
                    this.autoRanging.setTurretTracking(!autoRanging.isTurretTracking());
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_trigger"), Gamepad.class.getField("right_trigger"), Float::equals, (Float left_trigger, Float right_trigger) -> {
                if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                    float power = left_trigger - right_trigger;
                    container.launchDrive.setPower(power);
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_stick_button"), Gamepad.class.getField("right_stick_button"), Boolean::equals, (Boolean left_stick_button, Boolean right_stick_button) -> {
                if(left_stick_button && right_stick_button) {
                    int state = launcherControlMethodElements.indexOf(launcherControlMethod) + 1;
                    state %= launcherControlMethodElements.size();
                    launcherControlMethod = launcherControlMethodElements.get(state);

                    if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                        container.launchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        container.launchDrive.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                    } else {
                        container.launchDrive.setVelocity(0);
                        container.launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        autoRanging.setLauncherRunning(launcherControlMethod == LauncherControlMethod.ODOMETRY);
                    }
                }
            }));

        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void run() {
        container = hardware.getContainer();
        container.modules.childModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        autoRanging.setEnableDebugInfo(autoRangingDebugInfo);
        autoRanging.setLauncherRunning(true);

        super.run();
    }

    public void tick() {
        container.modules.childModule.module.clearBulkCache();
        container.tickBBs();
        String autoRangingTelem = autoRanging.tick(controlHubThread.robotPose.get());
        if(!autoRangingTelem.isEmpty()) {
            telemetry.append(autoRangingTelem);
        }

        Boolean autoShootState = null;

        switch(ballHandlingState) {
            case STOPPED:
            case OUTTAKE:
                break;

            case INTAKE:
                if(autoIntake.tick()) {
                    gamepad1.rumble(200);
                    ballHandlingState = BallHandlingState.STOPPED;
                }
                break;

            case SHOOTING:
                autoShootState = autoShoot.tick();
                break;
        }

        switch(autoGateCycleState) {
            case DISABLED:
                break;
            case TO_SHOOT_POS:
                if(!controlHubThread.isFollowerRunning() && !controlHubThread.isFollowerRequestPending()) {
                    toBallHandlingState(BallHandlingState.SHOOTING);
                    autoGateCycleState = AutoGateCycleState.SHOOTING;
                }
                break;
            case SHOOTING:
                if(Boolean.TRUE.equals(autoShootState)) {
                    toBallHandlingState(BallHandlingState.STOPPED);
                    autoGateCycleState = AutoGateCycleState.TO_GATE;
                    controlHubThread.requestFollowerFollow(activeAutoGateCyclePosePair.gateLineupPose == null ? List.of(gatePose) : List.of(activeAutoGateCyclePosePair.gateLineupPose, gatePose));
                }
                break;
            case TO_GATE:
                if(!controlHubThread.isFollowerRunning() && !controlHubThread.isFollowerRequestPending()) {
                    autoGateCycleState = AutoGateCycleState.DISABLED;
                }
                break;
        }

        telemetry.append("\nState: ");
        telemetry.append(ballHandlingState);
        telemetry.append("\nAutoShootState: ");
        telemetry.append(autoShootState);
        telemetry.append("\nIntake BB: ");
        telemetry.append(container.intakeBB.getState());
        telemetry.append("\nTransfer Middle BB: ");
        telemetry.append(container.transferMiddleBB.getState());
        telemetry.append("\nTransfer End BB: ");
        telemetry.append(container.transferEndBB.getState());
        telemetry.append("\n\n");

        container.voltageSensor.periodic();

        telemetry.append("Launcher Target Speed: ");
        telemetry.append(container.launchDrive.getTargetVelocity());
        telemetry.append("\nLauncher Current Speed: ");
        telemetry.append(container.launchDrive.getVelocity());
        telemetry.append("\nLauncher Control Method: ");
        telemetry.append(launcherControlMethod);
        telemetry.append("\n\n");

        updateTelemetry();
    }
}
