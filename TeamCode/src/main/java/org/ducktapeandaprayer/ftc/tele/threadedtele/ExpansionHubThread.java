package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;
import org.apache.commons.lang3.tuple.Pair;
import org.ducktapeandaprayer.ftc.auto.vision.AprilTagDetectionPipeline;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.TimedEvent;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class ExpansionHubThread extends ThreadEx {
    //
    // Dash options
    //

    public static long launcherVelocityCheckInterval = 200;
    public static double autoLaunchTimeout = 5.0;

    //
    // Globals
    //

    private final MainBotHardware hardware;

    public Boolean runVision = true;
    public Boolean visionDebugInfo = true;
    public Alliance alliance = Alliance.lastAlliance;

    private AprilTagDetectionPipeline pipeline;
    private String visionTelemetry = "";

    private enum AutoLaunchState {
        STOPPED,
        WAITING_ARTIFACT,
        SPINNING_UP,
        LAUNCHING,
        //For when we can't stop right now because we need to make sure that the ball exits the launcher successfully
        LAUNCHING_STOP_REQUESTED
    }

    private AutoLaunchState autoLaunchState = AutoLaunchState.STOPPED;

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

    public enum VisionComputationResultState {
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

    private VisionComputationResult computeLauncherSpeed() {
        if(!runVision) {
            return null;
        }

        double visionTargetSpeed = 0;
        VisionComputationResultState state = VisionComputationResultState.NO_TAG_FOUND;

        ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();

        StringBuilder visionDebugSb = new StringBuilder();

        int aprilTagID = alliance.toAprilTagID();

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

                double groundDist = AprilTagDetectionPipeline.getGroundDistance(targetDetection.pose);
                visionDebugSb.append("Ground distance: ").append(groundDist).append("\n");
                visionTargetSpeed = launchSpeedFromDist(groundDist);
                visionDebugSb.append("Launcher Vision Target Velocity: ");
                visionDebugSb.append(visionTargetSpeed).append("\n");

                if (Double.isNaN(visionTargetSpeed)) {
                    state = VisionComputationResultState.OUT_OF_RANGE;
                } else {
                    state = VisionComputationResultState.IN_RANGE;
                }

            } else {
                visionDebugSb.append("Target not found in frame\n");
            }

        } else {
            visionDebugSb.append("No detections\n");
        }

        visionDebugSb.append(pipeline.getTimingTelemetry()).append('\n');

        if(visionDebugInfo) {
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
                if((hardware.launchDrive.isAtTargetVelocity() && hardware.launchDrive.getTargetVelocity() != 0) || forceStart) {
                    hardware.blinkin.setPattern(MainBotHardware.Patterns.correctSpeed);
                } else {
                    hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedVisionLocked);
                }
                break;
        }

        if(hardware.launchDrive.getTargetVelocity() != 0) {
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

//                            updateLaunchSpeedAndBlinkinVision(computeLauncherSpeed());
//                            hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedNoVisionPattern);
    }

    private void stopAutoLaunch() {
        switch (autoLaunchState) {
            case STOPPED:
                throw new RuntimeException("Trying to stop auto launch while auto launch is already stopped");
            case WAITING_ARTIFACT:
                hardware.moverDrive.setPower(0);
                hardware.intakeDrive.setPower(0);
                break;
            case SPINNING_UP:
                //Nothing special needs to happen here
                break;
            case LAUNCHING:
                //Wait for speed dip instead of stopping the launcher right now to prevent any artifacts from getting stuck mid launch.
                RobotLog.ii("ExpansionHubThread", "Auto launch requested while waiting for speed dip");
                autoLaunchState = AutoLaunchState.LAUNCHING_STOP_REQUESTED;
                return;

            //Stop requested a second time while waiting for speed dip. We will e-stop
            case LAUNCHING_STOP_REQUESTED:
                hardware.kickerServo.setPosition(0);
                break;
        }

        stopLauncher();
        autoLaunchState = AutoLaunchState.STOPPED;
    }

    public ExpansionHubThread(MainBotHardware hardwareArg, Gamepad gamepad1, OptionDriver optionDriver) {
        this.hardware = hardwareArg;
        hardware.modules.childModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        try {
            optionDriver.add(new BooleanOption("Run Vision", "dpad_down", gamepad1, this.getClass().getField("runVision"), this));
            optionDriver.add(new BooleanOption("Vision debug info", "dpad_up", gamepad1, this.getClass().getField("visionDebugInfo"), this));
            optionDriver.add(new EnumOption<>("Alliance", "cross", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));

            //Movement
            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_bumper"), Gamepad.class.getField("right_bumper"), Boolean::equals, (Boolean left_bumper, Boolean right_bumper) -> {
                if(left_bumper == right_bumper) {
                    hardware.moverDrive.setPower(0);
                } else if(left_bumper) {
                    if(hardware.kickerServo.getPosition() == 0) {
                        hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
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
                    hardware.kickerServo.setPosition(0);
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("triangle"), (Boolean triangle) -> {
                if(triangle && launcherControlMethod != LauncherControlMethod.MANUAL) {
                    if(hardware.launchDrive.getTargetVelocity() == 0) {
                        startLauncher();
                    } else {
                        stopLauncher();
                    }
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("circle"), (Boolean circle) -> {
                if(circle) {
                    if(!hardware.launcherSwitch.getState()
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
                    if(power < 0) {
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
                        hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedNoVisionPattern);
                        hardware.launchDrive.setVelocity(0);
                    } else if(runVision) {
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
                    } else {
                        launcherControlMethod = LauncherControlMethod.MANUAL;
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hardware.launchDrive.setPower(hardware.intakeDrive.getPower());
                        hardware.intakeDrive.setPower(0);
                        hardware.blinkin.setPattern(MainBotHardware.Patterns.manualLaunchControl);
                    }
                }
            }));

            //Shoot three with autolaunch
            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("circle"), Boolean::equals, (Boolean circle) -> {
                if(!circle) {
                    return;
                }

                if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                    return;
                }

                if(autoLaunchState != AutoLaunchState.STOPPED) {
                    stopAutoLaunch();
                    return;
                }

                if(launcherControlMethod == LauncherControlMethod.VISION && computeLauncherSpeed().state == VisionComputationResultState.OUT_OF_RANGE) {
                    return;
                }

                startLauncher();

                //ADD: Lock out the rest of the controls while trying to auto launch

                //We check here because we don't want to start the mover if there is already and artifact in position and waiting
                if(hardware.launcherSwitch.getState()) {
                    hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
                    hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
                    autoLaunchState = AutoLaunchState.WAITING_ARTIFACT;
                } else {
                    autoLaunchState = AutoLaunchState.SPINNING_UP;
                }

                autoLaunchStartedWatingTime = runtime.seconds();
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
        if(runVision) {
            pipeline = new AprilTagDetectionPipeline(hardware);
            hardware.blinkin.setPattern(MainBotHardware.Patterns.tagNotFound);
            launcherControlMethod = LauncherControlMethod.VISION;
        } else {
            hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedNoVisionPattern);
            launcherControlMethod = LauncherControlMethod.PRESET;
        }

        super.run();
    }

    private double autoLaunchStartedWatingTime = Double.NaN;
    private int speedSamplesSeen = 0;
    private int samplesToLaunch = 0;

    public void tick() {
//        RobotLog.ii("ExpansionHubThread", "Running!");

        hardware.modules.childModule.module.clearBulkCache();
        telemetry.append(visionTelemetry);
        updateTelemetry();

        switch (autoLaunchState) {
            case STOPPED:
                break;

            case WAITING_ARTIFACT:
                if(hardware.launchDrive.isAtTargetVelocity()) {
                    speedSamplesSeen++;
                } else {
                    speedSamplesSeen = 0;
                }

                if(hardware.launcherSwitch.getState()) {
                    break;
                }

                hardware.moverDrive.setPower(0);
                hardware.intakeDrive.setPower(0);
                autoLaunchStartedWatingTime = runtime.seconds();
                autoLaunchState = AutoLaunchState.SPINNING_UP;

            case SPINNING_UP:
                if(hardware.launchDrive.isAtTargetVelocity()) {
                    speedSamplesSeen++;
                } else {
                    speedSamplesSeen = 0;
                }

                if (speedSamplesSeen < MainBotHardware.launchRequiredSpeedSamples) {
                    break;
                }

                hardware.kickerServo.setPosition(MainBotHardware.kickerServoUpPos);
                speedSamplesSeen = 0;
                autoLaunchStartedWatingTime = runtime.seconds();
                autoLaunchState = AutoLaunchState.LAUNCHING;

            case LAUNCHING:
            case LAUNCHING_STOP_REQUESTED:
                if(hardware.launchDrive.isAtTargetVelocity()) {
                    speedSamplesSeen = 0;
                } else {
                    speedSamplesSeen++;
                }

                if(speedSamplesSeen < MainBotHardware.launchRequiredSpeedSamples) {
                    break;
                }

                samplesToLaunch--;
                if(samplesToLaunch > 0 && autoLaunchState == AutoLaunchState.LAUNCHING) {
                    hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
                    hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
                    autoLaunchStartedWatingTime = runtime.seconds();
                    autoLaunchState = AutoLaunchState.WAITING_ARTIFACT;
                    speedSamplesSeen = 0;
                } else {
                    stopAutoLaunch();
                }
                break;


        }

        if(autoLaunchState != AutoLaunchState.STOPPED) {
            if(runtime.seconds() - autoLaunchStartedWatingTime > autoLaunchTimeout) {
                //We timed out, stop trying to autolaunch
                stopAutoLaunch();
            }
        }
    }
}
