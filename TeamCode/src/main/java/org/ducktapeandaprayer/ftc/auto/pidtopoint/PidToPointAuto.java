package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.ducktapeandaprayer.ftc.auto.roadrunner.RoadRunnerThread;
import org.ducktapeandaprayer.ftc.auto.RobotImplThread;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;

@Autonomous(name = "PID To Point Auto")
public class PidToPointAuto extends ThreadedOpMode {
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
}
