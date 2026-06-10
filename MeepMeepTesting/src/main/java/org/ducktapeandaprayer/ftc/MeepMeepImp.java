package org.ducktapeandaprayer.ftc;

import org.ducktapeandaprayer.ftc.auto.RobotInterface;

public class MeepMeepImp implements RobotInterface {
    @Override
    public void launch() {
        System.out.println("launch");
    }

    @Override
    public boolean launching() {
        return false;
    }

    @Override
    public void startLauncher() {
        System.out.println("launcher start");
    }

    @Override
    public void stopLauncher() {
        System.out.println("launcher stop");
    }

    @Override
    public void intake() {
        System.out.println("intaking");

    }

    @Override
    public void stopIntake() {
        System.out.println("intake stop");
    }
}
