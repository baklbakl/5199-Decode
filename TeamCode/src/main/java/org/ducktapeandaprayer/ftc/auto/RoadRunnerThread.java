package org.ducktapeandaprayer.ftc.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.ducktapeandaprayer.ftc.auto.options.Delay;
import org.ducktapeandaprayer.ftc.auto.options.GatePush;
import org.ducktapeandaprayer.ftc.auto.options.StartPos;
import org.ducktapeandaprayer.ftc.auto.roadrunner.MecanumDrive;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.IntOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

public class RoadRunnerThread extends ThreadEx {
    private final MainBotHardware hardware;
    private SplitAuto splitAuto;
    private Action path = null;
    private TrajectoryActionBuilder builder = null;
    private MecanumDrive mecanumDrive;

    public StartPos startPos;
    public Delay delay;
    public boolean score;
    public int collectCount;
    public boolean park;
    public GatePush gate;

    public RoadRunnerThread(MainBotHardware hardware, OptionDriver optionDriver, Gamepad gamepad1, RobotImplThread robot) {
        this.hardware = hardware;
        this.hardware.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        try {
            optionDriver.add(new EnumOption<>("Start Pos", "dpad_down", gamepad1, this.getClass().getField("startPos"), this, StartPos.class));
            optionDriver.add(new EnumOption<>("Delay", "cross", gamepad1, this.getClass().getField("delay"), this, Delay.class));
            optionDriver.add(new BooleanOption("Score", "circle", gamepad1, this.getClass().getField("score"), this));
            optionDriver.add(new IntOption("Collect Count", "square", gamepad1, this.getClass().getField("collectCount"), this, 0, 3));
            optionDriver.add(new BooleanOption("Park", "triangle", gamepad1, this.getClass().getField("park"), this));
            optionDriver.add(new EnumOption<>("Push Gate", "dpad_left", gamepad1, this.getClass().getField("gate"), this, GatePush.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        splitAuto = new SplitAuto(
                (startPose) -> {
                    mecanumDrive = new MecanumDrive(hardware, startPose, robot::getVoltage);
                    return mecanumDrive.actionBuilder(startPose);
                },
                new RobotImplThread(hardware)
        );
    }

    @Override
    public void tick() {

    }

    @Override
    public void run() {


        super.run();
    }
}
