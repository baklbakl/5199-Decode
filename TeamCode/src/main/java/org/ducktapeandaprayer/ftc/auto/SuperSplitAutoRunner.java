package org.ducktapeandaprayer.ftc.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;

@Autonomous(name = "Super Split Auto")
public class SuperSplitAutoRunner extends ThreadedOpMode {
    @Override
    protected void init_internal() {
        super.init_internal();

        MainBotHardware hardware = MainBotHardware.getOrCreate(hardwareMap);

        RobotImplThread robotImpl = new RobotImplThread(hardware);
        RoadRunnerThread roadRunnerThread = new RoadRunnerThread(hardware, this, robotImpl);
        super.getThreads().put("RoadRunnerThread", roadRunnerThread);
        super.getThreads().put("RobotImplThread", robotImpl);
    }
}
