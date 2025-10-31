package net.polarbub.ftc.auto;

public interface RobotInterface {
    void setLauncherSpeed(double velocity);
    boolean isLauncherSpunUp();
    void runIntake(IntakeDirection direction);
    void runMoverServos(IntakeDirection direction);
}
