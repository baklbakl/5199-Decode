package net.polarbub.ftc.auto;

import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.MainBotHardware;

public class RobotImplementation implements RobotInterface {
    private final MainBotHardware hardware;

    public RobotImplementation(MainBotHardware hardware) {
        this.hardware = hardware;
    }

    @Override
    public void setLauncherSpeed(double velocity) {
        hardware.launchDrive.setVelocity(velocity);
    }

    @Override
    public boolean isLauncherSpunUp() {
        return Math.abs(hardware.launchDrive.getVelocity() - hardware.launchDrive.getTargetVelocity()) < MainBotConfig.MoveConfig.motorPositionTolerance;
    }

    @Override
    public void runIntake(IntakeDirection direction) {
        double power;

        switch(direction) {
            case IN:
                power = 1;
                break;
            case OUT:
                power = -1;
                break;
            case STOP:
                power = 0;
                break;
            default:
                throw new IllegalArgumentException("Unknown power for direction \"" + direction + "\"");
        }

        hardware.intakeDrive.setPower(power);
    }

    @Override
    public void runMoverServos(IntakeDirection direction) {
        double power;

        switch(direction) {
            case IN:
                power = 1;
                break;
            case OUT:
                power = 0;
                break;
            case STOP:
                power = MainBotConfig.MoveConfig.restingServoPos;
                break;
            default:
                throw new IllegalArgumentException("Unknown servo position for direction \"" + direction + "\"");
        }

        hardware.moverServo.setPosition(power);
        hardware.moverServo2.setPosition(power);
    }
}
