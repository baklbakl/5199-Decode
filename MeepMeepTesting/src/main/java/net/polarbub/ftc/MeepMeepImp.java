package net.polarbub.ftc;

import net.polarbub.ftc.auto.RobotInterface;

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
