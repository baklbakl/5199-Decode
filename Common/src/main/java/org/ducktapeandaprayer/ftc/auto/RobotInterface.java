package org.ducktapeandaprayer.ftc.auto;

public interface RobotInterface {
    void launch(int numToLaunch);
    void setLauncherSpeed(double speed);
    void intake();
    void stopIntake();
}
