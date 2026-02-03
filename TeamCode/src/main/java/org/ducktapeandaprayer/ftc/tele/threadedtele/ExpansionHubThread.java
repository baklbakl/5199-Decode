package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.apache.commons.lang3.tuple.Pair;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.hardware.AutoLaunch;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.TimedEvent;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.ducktapeandaprayer.ftc.vision.AprilTagPoseEx;

public class ExpansionHubThread extends ThreadEx {
    //
    // Dash options
    //

    public static long launcherVelocityCheckInterval = 200;

    //
    // Globals
    //

    public Boolean autoSpeedDebInfo = true;

    private final MainBotHardware hardware;
    private final ThreadedTele threadedTele;
    private final AutoLaunch autoLaunch;

    private String visionTelemetry = "";

    private enum LauncherControlMethod {
        MANUAL,
        VISION,
        PRESET
    }

    private LauncherControlMethod launcherControlMethod;

    private static double launchSpeedFromDist(double dist) {
        synchronized (MainBotHardware.launchSpeedMapping) {
            Pair<Double, Double> lowestPair = MainBotHardware.launchSpeedMapping.get(0);
            if(dist < lowestPair.getKey()) {
                return Double.NaN;
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


        return Double.NaN;
    }

    private enum VisionComputationResultState {
        NO_TAG_FOUND,
        OUT_OF_RANGE,
        IN_RANGE
    }

    private static class VisionComputationResult {
        public final VisionComputationResultState state;
        public final double speed;

        private VisionComputationResult(VisionComputationResultState state, double speed) {
            this.state = state;
            this.speed = speed;
        }
    }

    //This could realize when it has seen this pose before and skip its processing, but that would add complexity and recomputing is easier.
    private VisionComputationResult computeLauncherSpeed() {
        if(!threadedTele.runVision) {
            return null;
        }

        StringBuilder visionDebugSb = new StringBuilder();

        VisionComputationResultState state = null;
        double groundDist = Double.NaN;

        synchronized (threadedTele.tagPoseLock) {
            AprilTagPoseEx pose = threadedTele.getTagPose();

            if(pose == null) {
                state = VisionComputationResultState.NO_TAG_FOUND;
            } else {
                groundDist = pose.groundDistance;
            }
        }

        double visionTargetSpeed = launchSpeedFromDist(groundDist);
        visionDebugSb.append("Launcher Vision Target Velocity: ");
        visionDebugSb.append(visionTargetSpeed).append("\n");

        if(state != VisionComputationResultState.NO_TAG_FOUND) {
            if (Double.isNaN(visionTargetSpeed)) {
                state = VisionComputationResultState.OUT_OF_RANGE;
            } else {
                state = VisionComputationResultState.IN_RANGE;
            }
        }

        if(autoSpeedDebInfo) {
            visionTelemetry = visionDebugSb.toString();
        }

        return new VisionComputationResult(state, visionTargetSpeed);
    }

    private void updateLaunchSpeedAndBlinkinVision(VisionComputationResult result) {
        updateLaunchSpeedAndBlinkinVision(result, false);
    }

    private void updateLaunchSpeedAndBlinkinVision(VisionComputationResult result, boolean forceStart) {
        if(launcherControlMethod != LauncherControlMethod.VISION) {
            throw new RuntimeException("Tried to run a vision only function in non vision control mode");
        }

        double targetSpeed = 0;
        switch (result.state) {
            case NO_TAG_FOUND:
                targetSpeed = MainBotHardware.teleOpLauncherNoVisionVelocity;
                hardware.blinkin.setPattern(MainBotHardware.Patterns.tagNotFound);

                break;
            case OUT_OF_RANGE:
                targetSpeed = MainBotHardware.teleOpLauncherNoVisionVelocity;
                hardware.blinkin.setPattern(MainBotHardware.Patterns.outOfLaunchRange);

                break;
            case IN_RANGE:
                targetSpeed = result.speed;
                if(hardware.launchDrive.isAtTargetVelocity() && hardware.launchDrive.getTargetVelocity() != 0) {
                    hardware.blinkin.setPattern(MainBotHardware.Patterns.correctSpeed);
                } else {
                    hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedVisionLocked);
                }
                break;
        }

        if(hardware.launchDrive.getTargetVelocity() != 0 || forceStart) {
            hardware.launchDrive.setVelocity(targetSpeed);
        }
    }

    private void startLauncher() {
        if(launcherControlMethod == LauncherControlMethod.VISION) {
            updateLaunchSpeedAndBlinkinVision(computeLauncherSpeed(), true);
        } else if(launcherControlMethod == LauncherControlMethod.PRESET) {
            hardware.launchDrive.setVelocity(MainBotHardware.teleOpLauncherNoVisionVelocity);
        } else {
            throw new RuntimeException("Oops unaccounted for launcher control method \"" + launcherControlMethod + "\"");
        }
    }

    private void stopLauncher() {
        hardware.launchDrive.setVelocity(0);
        if(launcherControlMethod == LauncherControlMethod.VISION) {
            updateLaunchSpeedAndBlinkinVision(computeLauncherSpeed());
        } else if(launcherControlMethod == LauncherControlMethod.PRESET) {
            hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedNoVisionPattern);
        } else {
            throw new RuntimeException("Oops unaccounted for launcher control method \"" + launcherControlMethod + "\"");
        }
    }

    public ExpansionHubThread(MainBotHardware hardwareArg, ThreadedTele threadedTele, ControlHubThread controlHubThread, Gamepad gamepad1, OptionDriver optionDriver) {
        this.hardware = hardwareArg;
        this.threadedTele = threadedTele;

        this.autoLaunch = new AutoLaunch(
                hardware,
                this::startLauncher,
                () -> {
                    stopLauncher();
                    if(controlHubThread.getAutoAimState() != ControlHubThread.AutoAimState.STOPPED) {
                        controlHubThread.requestAutoAimStartStop(false);
                    }
                },
                //Disabled or locked auto aim are safe for launch. We maybe could check for auto speed lock, but it hasn't been an issue, so we won't
                () -> {
                    //ADD: Check that auto speed is locked if we're using it before letting auto launch happen
                    ControlHubThread.AutoAimState state = controlHubThread.getAutoAimState();
                    return state == ControlHubThread.AutoAimState.LOCKED || state == ControlHubThread.AutoAimState.DISABLED;
                }
        );
        hardware.modules.childModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        try {
            optionDriver.add(new BooleanOption("Auto Speed debug info", "dpad_right", gamepad1, this.getClass().getField("autoSpeedDebInfo"), this));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_bumper"), Gamepad.class.getField("right_bumper"), Boolean::equals, (Boolean left_bumper, Boolean right_bumper) -> {
                if(autoLaunch.isLaunching()) {
                    return;
                }

                if(left_bumper == right_bumper) {
                    hardware.moverDrive.setPower(0);
                } else if(left_bumper) {
                    if(hardware.kickerServo.getPosition() == MainBotHardware.kickerServoDownPos) {
                        hardware.moverDrive.setPower(hardware.launcherSwitch.getState() ? MainBotHardware.moverDrivePower : MainBotHardware.moverDriveBallInPower);
                    }
                } else if(right_bumper) {
                    hardware.moverDrive.setPower(-MainBotHardware.moverDrivePower);
                } else {
                    throw new RuntimeException("Oopsy. Something bad happened. Never should be here");
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("options"), (Boolean options) -> {
                if(options) {
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoUpPos);
                    hardware.moverDrive.setPower(0);
                } else if (!gamepad1.circle){
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoDownPos);
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("triangle"), (Boolean triangle) -> {
                if(triangle && launcherControlMethod != LauncherControlMethod.MANUAL && !autoLaunch.isLaunching()) {
                    if(hardware.launchDrive.getTargetVelocity() == 0) {
                        startLauncher();
                    } else {
                        stopLauncher();
                    }
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("circle"), (Boolean circle) -> {
                if(autoLaunch.isLaunching()) {
                    return;
                }

                if(circle) {
                    if(!hardware.launcherSwitch.getState()
                            && hardware.launchDrive.isAtTargetVelocity()
                            && hardware.launchDrive.getTargetVelocity() != 0
                    ) {
                        hardware.kickerServo.setPosition(MainBotHardware.kickerServoUpPos);
                        hardware.moverDrive.setPower(0);
                    }
                } else if(!gamepad1.options) {
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoDownPos);
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_trigger"), Gamepad.class.getField("right_trigger"), Float::equals, (Float left_trigger, Float right_trigger) -> {
                float power = left_trigger - right_trigger;
                if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                    if(power > 0) {
                        power /= 2;
                    }
                    hardware.launchDrive.setPower(power);
                } else if(!autoLaunch.isLaunching()) {
                    hardware.intakeDrive.setPower(power);
                }

            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_stick_button"), Boolean::equals, (Boolean left_stick_button) -> {
                if(left_stick_button) {
                    if(launcherControlMethod == LauncherControlMethod.VISION) {
                        launcherControlMethod = LauncherControlMethod.PRESET;
                        hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedNoVisionPattern);
                        hardware.launchDrive.setVelocity(0);
                    } else if(this.threadedTele.runVision) {
                        launcherControlMethod = LauncherControlMethod.VISION;
                        hardware.launchDrive.setVelocity(0);
                        updateLaunchSpeedAndBlinkinVision(computeLauncherSpeed());
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
                        hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedNoVisionPattern);
                    } else if(!autoLaunch.isLaunching()) {
                        launcherControlMethod = LauncherControlMethod.MANUAL;
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hardware.launchDrive.setPower(hardware.intakeDrive.getPower());
                        hardware.intakeDrive.setPower(0);
                        hardware.blinkin.setPattern(MainBotHardware.Patterns.manualLaunchControl);
                    }
                }
            }));

            //Shoot three with autolaunch
            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("cross"), Boolean::equals, (Boolean cross) -> {
                if(!cross) {
                    return;
                }

                if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                    return;
                }

                if(autoLaunch.isLaunching()) {
                    autoLaunch.stopAutoLaunch();
                    return;
                }

                if(launcherControlMethod == LauncherControlMethod.VISION && computeLauncherSpeed().state == VisionComputationResultState.OUT_OF_RANGE) {
                    return;
                }

                if(controlHubThread.getAutoAimState() == ControlHubThread.AutoAimState.STOPPED) {
                    controlHubThread.requestAutoAimStartStop(true);
                }
                autoLaunch.startAutoLaunching(3);
            }));


        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        timedEvents.add(new TimedEvent(launcherVelocityCheckInterval, () -> {
//            System.out.println("Run vision: " + runVision);
//            System.out.println("Run vision deb info: " + visionDebugInfo);

            VisionComputationResult result = computeLauncherSpeed();

            if(launcherControlMethod == LauncherControlMethod.VISION) {
                updateLaunchSpeedAndBlinkinVision(result);

            } else if(launcherControlMethod == LauncherControlMethod.PRESET) {
                if(hardware.launchDrive.isAtTargetVelocity() && hardware.launchDrive.getTargetVelocity() != 0) {
                    hardware.blinkin.setPattern(MainBotHardware.Patterns.correctSpeedNoVision);
                } else {
                    hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedNoVisionPattern);
                }
            }
        }));
    }

    @Override
    public void run() {
        if(threadedTele.runVision) {
            hardware.blinkin.setPattern(MainBotHardware.Patterns.tagNotFound);
            launcherControlMethod = LauncherControlMethod.VISION;
        } else {
            hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedNoVisionPattern);
            launcherControlMethod = LauncherControlMethod.PRESET;
        }

        super.run();
    }

    public void tick() {
//        RobotLog.ii("ExpansionHubThread", "Running!");
        autoLaunch.tick();

        if(hardware.moverDrive.getPower() > 0) {
            if(hardware.launcherSwitch.getState()) {
                hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
            } else {
                hardware.moverDrive.setPower(MainBotHardware.moverDriveBallInPower);
            }
        }

        hardware.modules.childModule.module.clearBulkCache();

        telemetry.append(visionTelemetry);

        telemetry.append("Launcher Target Speed: ");
        telemetry.append(hardware.launchDrive.getTargetVelocity());
        telemetry.append("\nLauncher Current Speed: ");
        telemetry.append(hardware.launchDrive.getVelocity());
        updateTelemetry();
    }
}
