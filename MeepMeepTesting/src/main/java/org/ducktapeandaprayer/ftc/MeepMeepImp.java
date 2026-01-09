package org.ducktapeandaprayer.ftc;

import org.ducktapeandaprayer.ftc.auto.RobotInterface;

public class MeepMeepImp implements RobotInterface {


    @Override
    public void launch(int numToLaunch, double speed) {
      System.out.println("launch") ;
    }

    @Override
    public void intake() {
        System.out.println("intaking");

    }

    @Override
    public void stopIntake() {

    }
}
