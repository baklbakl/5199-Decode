package org.ducktapeandaprayer.ftc.auto.roadrunner;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.ducktapeandaprayer.ftc.auto.RobotImplThread;
import org.ducktapeandaprayer.ftc.auto.SplitAuto;
import org.ducktapeandaprayer.ftc.auto.options.Delay;
import org.ducktapeandaprayer.ftc.auto.options.GatePush;
import org.ducktapeandaprayer.ftc.auto.options.StartPos;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.action.ActionThreadEx;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.IntOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.lang.Math;

@Autonomous(name = "Split Auto Action Thread")
public class SplitAutoActionThread extends ThreadedOpMode {
    private MainBotHardware hardware = null;
    private MainBotHardware.Container container = null;
    private RobotImplThread robotImpl = null;
    private MecanumDrive mecanumDrive = null;
    private SplitAuto splitAuto = null;
    private Action action = null;
    private TrajectoryActionBuilder builder = null;

    public StartPos startPos;
    public Delay delay;
    public boolean score;
    public int collectCount;
    public boolean park;
    public GatePush gate;

    @Override
    protected void init_internal() {
        super.init_internal();

        hardware = MainBotHardware.getOrCreate(hardwareMap);
        container = hardware.getContainer();

        robotImpl = new RobotImplThread(hardware);
        super.getThreads().put("RobotImplThread", robotImpl);

        container.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        OptionDriver optionDriver = this.getOptionDriver();

        try {
            optionDriver.add(new EnumOption<>("Start Pos", "dpad_down", gamepad1, this.getClass().getField("startPos"), this, StartPos.class));
            optionDriver.add(new EnumOption<>("Delay", "dpad_up", gamepad1, this.getClass().getField("delay"), this, Delay.class));
            optionDriver.add(new BooleanOption("Score", "circle", gamepad1, this.getClass().getField("score"), this));
            optionDriver.add(new IntOption("Collect Count", "square", gamepad1, this.getClass().getField("collectCount"), this, 0, 3));
            optionDriver.add(new BooleanOption("Park", "dpad_right", gamepad1, this.getClass().getField("park"), this));
            optionDriver.add(new EnumOption<>("Push Gate", "dpad_left", gamepad1, this.getClass().getField("gate"), this, GatePush.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        splitAuto = new SplitAuto(
                (startPose) -> {
                    mecanumDrive = new MecanumDrive(hardware, startPose, robotImpl::getVoltage, this::isStopRequested);
                    return mecanumDrive.actionBuilder(startPose);
                },
                robotImpl
        );
    }

    @Override
    public String optionDriverUpdateCallback() {
        builder = splitAuto.fillBuilder(delay, startPos, getAlliance(), score, collectCount, park, gate);
        Action path = builder.build();

        action = new RaceAction(
                path,
                (TelemetryPacket telemetryPacket) -> {
                    container.modules.parentModule.module.clearBulkCache();
                    container.voltageSensor.periodic();
                    telemetryPacket.put("x", mecanumDrive.localizer.getPose().position.x);
                    telemetryPacket.put("y", mecanumDrive.localizer.getPose().position.y);
                    telemetryPacket.put("heading (deg)", Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
                    return true;
                }
        );

        return "\nDone building paths";
    }

    @Override
    protected void start_internal() {
        super.getThreads().put("Action Thread", new ActionThreadEx(action, this));
        super.start_internal();
    }
}

