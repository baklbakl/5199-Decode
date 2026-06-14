package org.ducktapeandaprayer.ftc.auto.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.ducktapeandaprayer.ftc.auto.RobotImplThread;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;

//FIX: Delete super split auto after testing split auto action thread tele crossover
@Autonomous(name = "Super Split Auto")
//FIX: This doesn't auto end anymore
public class SuperSplitAutoRunner extends ThreadedOpMode {
    @Override
    protected void init_internal() {
        super.init_internal();

        MainBotHardware hardware = MainBotHardware.getOrCreate(hardwareMap);
        hardware.getContainer();

        RobotImplThread robotImpl = new RobotImplThread(hardware);
        RoadRunnerThread roadRunnerThread = new RoadRunnerThread(hardware, this, robotImpl);
        super.getThreads().put("RoadRunnerThread", roadRunnerThread);
        super.getThreads().put("RobotImplThread", robotImpl);
    }

    @Override
    protected void start_internal() {
        Alliance.lastAlliance = getAlliance();
        super.start_internal();
    }
}