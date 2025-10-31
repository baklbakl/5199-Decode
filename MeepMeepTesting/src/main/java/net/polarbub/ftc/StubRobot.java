package net.polarbub.ftc;

import net.polarbub.ftc.auto.IntakeDirection;
import net.polarbub.ftc.auto.RobotInterface;

public class StubRobot implements RobotInterface {
    @Override
    public void setLauncherSpeed(double velocity) {}

    @Override
    public boolean isLauncherSpunUp() {
        return true;
    }

    @Override
    public void runIntake(IntakeDirection direction) {}

    @Override
    public void runMoverServos(IntakeDirection direction) {}
}
