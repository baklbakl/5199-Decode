package org.ducktapeandaprayer.ftc.auto;

import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;

import java.util.function.Supplier;

public class RobotImpl implements RobotInterface {
    private final MainBotHardware hardware;
    private final Supplier<Boolean> isOpModeActive;

    public RobotImpl(MainBotHardware hardware, Supplier<Boolean> isOpModeActive) {
        this.hardware = hardware;
        this.isOpModeActive = isOpModeActive;
    }

    @Override
    public void launch(int numToLaunch) {
        //FIX: Add auto launch stuff
    }

    @Override
    public void setLauncherSpeed(double speed) {
        hardware.launchDrive.setVelocity(speed);
    }

    @Override
    public void intake() {
        hardware.transferDrive.setPower(MainBotHardware.transferInPower);
        hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
    }

    @Override
    public void stopIntake() {
        hardware.intakeDrive.setPower(0);
        hardware.transferDrive.setPower(0);
    }
}
