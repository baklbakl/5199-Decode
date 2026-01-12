package org.ducktapeandaprayer.ftc.auto;

import org.ducktapeandaprayer.ftc.hardware.AutoLaunch;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;

import java.util.function.Supplier;

public class RobotImpl implements RobotInterface {
    private final MainBotHardware hardware;
    private final AutoLaunch autoLaunch;
    private final Supplier<Boolean> isOpModeActive;

    public RobotImpl(MainBotHardware hardware, Supplier<Boolean> isOpModeActive) {
        this.hardware = hardware;
        this.isOpModeActive = isOpModeActive;
        autoLaunch = new AutoLaunch(hardware);
    }

    @Override
    public void launch(int numToLaunch, double speed) {
        autoLaunch.startAutoLaunching(3, speed);

        while(autoLaunch.isLaunching() && isOpModeActive.get()) {
            autoLaunch.tick();
        }
    }

    @Override
    public void startLauncher(double speed) {
        hardware.launchDrive.setVelocity(speed);
    }

    @Override
    public void intake() {
        hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
        hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
    }

    @Override
    public void stopIntake() {
        hardware.intakeDrive.setPower(0);
        hardware.moverDrive.setPower(0);
    }
}
