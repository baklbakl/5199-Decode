package net.polarbub.ftc;

import net.polarbub.ftc.auto.RobotInterface;

import java.awt.Robot;

public class MeepMeepImp implements RobotInterface {


    @Override
    public void launch() {
      System.out.println("launch") ;
    }

    @Override
    public void intake() {
        System.out.println("intaking");

    }

//    public int getSpike()
//    {
//        return spike;
//    }
}
