package org.ducktapeandaprayer.ftc.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ducktapeandaprayer.ftc.hardware.BeamBreak;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;

import java.util.concurrent.atomic.AtomicReference;

public class RobotImplThread extends ThreadEx implements RobotInterface {
    public static int speedDipThresh = 200;

    private final MainBotHardware hardware;
    private final ElapsedTime timer = new ElapsedTime();
    private double shootingStartTime = Double.NaN;

    private volatile boolean requestLaunchingStart = false;
    private volatile boolean launching = false;
    private final AtomicReference<Double> requestLauncherSpeed = new AtomicReference<>(Double.NaN);
    private final AtomicReference<Double> requestIntakeSpeed = new AtomicReference<>(Double.NaN);
    private final AtomicReference<Double> voltage = new AtomicReference<>(Double.NaN);

    public RobotImplThread(MainBotHardware hardware) {
        this.hardware = hardware;
        this.hardware.modules.childModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    @Override
    public void run() {
        super.run();
    }

    @Override
    public void tick() {
        hardware.modules.childModule.module.clearBulkCache();
        hardware.tickBBs();

        voltage.set(hardware.voltageSensor.getVoltage());

        if(requestLaunchingStart) {
            if(launching) {
                throw new RuntimeException("Can't start lauching while launching, but stupid because we didn't catch it outside this thread");
            }

            shootingStartTime = timer.time();
            hardware.rampServo.setPosition(MainBotHardware.rampShootPos);
            hardware.transferDrive.setPower(0);
            hardware.intakeDrive.setPower(0);
            launching = true;
            requestLaunchingStart = false;
            //ADD: Check to make sure we have some artifacts before starting launching. Also add a launching timeout
        }

        if(launching) {
            if(timer.seconds() - shootingStartTime >= MainBotHardware.shootTransferOnDelay) {
                hardware.transferDrive.setPower(MainBotHardware.transferInPower);
                hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
                shootingStartTime = Double.NaN;
            }

            if(hardware.intakeBB.getState() == BeamBreak.State.EMPTY && hardware.transferMiddleBB.getState() == BeamBreak.State.EMPTY) {
                hardware.rampServo.setPosition(MainBotHardware.rampLastBallPos);
            }

            //Deal with dumb rounding
            if(Math.abs(hardware.rampServo.getPosition() - MainBotHardware.rampLastBallPos) < 0.01 && hardware.launchDrive.getTargetVelocity() - hardware.launchDrive.getVelocity() > speedDipThresh) {
                launching = false;
                hardware.transferDrive.setPower(0);
                hardware.intakeDrive.setPower(0);
                hardware.rampServo.setPosition(MainBotHardware.rampDownPos);
            }
        //We must be intaking or idling in which case then we should stop the intake and transfer when they are full
        } else if(hardware.transferMiddleBB.getState() == BeamBreak.State.FULL && hardware.transferEndBB.getState() == BeamBreak.State.FULL) {
            hardware.transferDrive.setPower(0);

            if(hardware.intakeBB.getState() == BeamBreak.State.FULL) {
                hardware.intakeDrive.setPower(0);
            }

        }

        synchronized (requestLauncherSpeed) {
            double speed = requestLauncherSpeed.get();
            if(!Double.isNaN(speed)) {
                hardware.launchDrive.setVelocity(speed);
                requestLauncherSpeed.set(Double.NaN);
            }
        }

        synchronized (requestIntakeSpeed) {
            double speed = requestIntakeSpeed.get();
            if(!Double.isNaN(speed)) {
                hardware.intakeDrive.setPower(speed);
                hardware.transferDrive.setPower(speed);
                requestIntakeSpeed.set(Double.NaN);
            }
        }
    }

    @Override
    public void launch() {
        if(launching) {
            throw new RuntimeException("Can't start lauching while launching");
        }
        requestLaunchingStart = true;

    }

    @Override
    public boolean launching() {
        return launching;
    }

    @Override
    public void startLauncher() {
        requestLauncherSpeed.set(MainBotHardware.launcherPresetVelocity);
    }

    @Override
    public void stopLauncher() {
        requestLauncherSpeed.set(0.0);
    }

    @Override
    public void intake() {
        if(launching) {
            throw new RuntimeException("Can't mess with intake while launching");
        }
        requestIntakeSpeed.set(MainBotHardware.intakeInPower);
    }

    @Override
    public void stopIntake() {
        if(launching) {
            throw new RuntimeException("Can't mess with intake while launching");
        }
        requestIntakeSpeed.set(0.0);
    }

    public double getVoltage() {
        return voltage.get();
    }
}
