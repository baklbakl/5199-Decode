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
        autoLaunch = new AutoLaunch(hardware, () -> {}, () -> {},
                () -> hardware.launchDrive.getTargetVelocity() != 0);
    }

    @Override
    public void launch(int numToLaunch) {
        autoLaunch.startAutoLaunching(3);

        while(autoLaunch.isLaunching() && isOpModeActive.get()) {
            autoLaunch.tick();
        }
    }

    @Override
    public void setLauncherSpeed(double speed) {
        hardware.launchDrive.setVelocity(speed);
    }

    @Override
    public void intake() {
        hardware.moverDrive.setPower(MainBotHardware.moverDriveAutoPower);
        hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
    }

    @Override
    public void stopIntake() {
        hardware.intakeDrive.setPower(0);
        hardware.moverDrive.setPower(0);
    }
}
