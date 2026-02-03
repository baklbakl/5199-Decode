package org.ducktapeandaprayer.ftc.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.tele.threadedtele.ExpansionHubThread;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoLaunch {
    public static double autoLaunchTimeout = 2.0;

    private enum State {
        STOPPED,
        WAITING_ARTIFACT,
        SPINNING_UP,
        LAUNCHING,
        //For when we shouldn't stop right now because we need to make sure that the ball exits the launcher successfully
        LAUNCHING_STOP_REQUESTED
    }

    private final MainBotHardware hardware;
    private final Runnable stopLauncher;
    private final Runnable startLauncher;
    private final Supplier<Boolean> launchReady;
    private final ElapsedTime timeout = new ElapsedTime();

    private State state = State.STOPPED;
    private int speedSamplesSeen = 0;
    private int artifactsToLaunch = 0;

    //Stop and start launcher are called at the start and end of auto launching, so they can also be used to do other cleanup
    public AutoLaunch(MainBotHardware hardware, Runnable startLauncher, Runnable stopLauncher, Supplier<Boolean> launchReady) {
        this.hardware = hardware;
        this.stopLauncher = stopLauncher;
        this.startLauncher = startLauncher;
        this.launchReady = launchReady;
    }

    public AutoLaunch(MainBotHardware hardware) {
        this.hardware = hardware;
        this.startLauncher = null;
        this.stopLauncher = () -> hardware.launchDrive.setVelocity(0);
        this.launchReady = () -> true;
    }

    private void internalStartAutolaunching(int count, Runnable startLauncherReal) {
        if(state != State.STOPPED) {
            throw new IllegalStateException("Not allowed to start launching while launching is in progress");
        }

        artifactsToLaunch = count;
        startLauncherReal.run();

        //We check here because we don't want to start the mover if there is already and artifact in position and waiting
        if(hardware.launcherSwitch.getState()) {
            hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
            hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
            state = State.WAITING_ARTIFACT;
        } else {
            hardware.moverDrive.setPower(0);
            hardware.intakeDrive.setPower(0);
            state = State.SPINNING_UP;
        }
        hardware.kickerServo.setPosition(MainBotHardware.kickerServoDownPos);

        timeout.reset();
    }

    public void startAutoLaunching(int count, double velocity) {
        internalStartAutolaunching(count, () -> hardware.launchDrive.setVelocity(velocity));
    }

    public void startAutoLaunching(int count) {
        if(startLauncher == null) {
            throw new IllegalStateException("Must have provided a start launcher runnable on construction to use startAutoLaunching without a speed");
        }

        internalStartAutolaunching(count, startLauncher);
    }

    public void stopAutoLaunch() {
        switch (state) {
            case STOPPED:
                throw new RuntimeException("Trying to stop auto launch while auto launch is already stopped");

            case LAUNCHING:
                //Wait for speed dip instead of stopping the launcher right now to prevent any artifacts from getting stuck mid launch.
                RobotLog.ii("AutoLaunch", "Auto launch stop requested while waiting for speed dip");
                state = State.LAUNCHING_STOP_REQUESTED;
                return;
            //Stop requested a second time while waiting for speed dip. We will e-stop
            case LAUNCHING_STOP_REQUESTED:
                hardware.kickerServo.setPosition(MainBotHardware.kickerServoDownPos);
            case SPINNING_UP:
            case WAITING_ARTIFACT:
                hardware.moverDrive.setPower(0);
                hardware.intakeDrive.setPower(0);
                stopLauncher.run();
                break;
        }

        state = State.STOPPED;
    }

    public void tick() {
        switch (state) {
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
                timeout.reset();
                state = State.SPINNING_UP;

            case SPINNING_UP:
                if(hardware.launchDrive.isAtTargetVelocity()) {
                    speedSamplesSeen++;
                } else {
                    speedSamplesSeen = 0;
                }

                if (speedSamplesSeen < MainBotHardware.launchRequiredSpeedSamples || !launchReady.get()) {
                    break;
                }

                hardware.kickerServo.setPosition(MainBotHardware.kickerServoUpPos);
                speedSamplesSeen = 0;
                timeout.reset();
                state = State.LAUNCHING;

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

                artifactsToLaunch--;
                if(artifactsToLaunch > 0 && state == State.LAUNCHING) {
                    hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
                    hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoDownPos);
                    timeout.reset();
                    state = State.WAITING_ARTIFACT;
                    speedSamplesSeen = 0;
                } else {
                    state = State.LAUNCHING_STOP_REQUESTED;
                    stopAutoLaunch();
                }
                break;
        }

        if(state != State.STOPPED) {
            if(timeout.seconds() > autoLaunchTimeout) {
                //We timed out, stop trying to autolaunch.
                if(state == State.LAUNCHING) {
                    state = State.LAUNCHING_STOP_REQUESTED;
                }

                stopAutoLaunch();
            }
        }
    }

    public boolean isLaunching() {
        return state != State.STOPPED;
    }
}
