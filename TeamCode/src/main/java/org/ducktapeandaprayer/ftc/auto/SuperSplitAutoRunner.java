package org.ducktapeandaprayer.ftc.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.ducktapeandaprayer.ftc.auto.options.Delay;
import org.ducktapeandaprayer.ftc.auto.options.GatePush;
import org.ducktapeandaprayer.ftc.auto.options.StartPos;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.auto.roadrunner.MecanumDrive;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.IntOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.lang.Math;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Super Split Auto")
public class SuperSplitAutoRunner extends ThreadedOpMode {
    private MainBotHardware hardware;
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

    @Override
    protected void init_internal() {
        super.init_internal();
        OptionDriver optionDriver = super.getOptionDriver();
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

        hardware = MainBotHardware.getOrCreate(hardwareMap);
        hardware.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        RobotImplThread robotImpl = new RobotImplThread(hardware);

        splitAuto = new SplitAuto(
                (startPose) -> {
                    mecanumDrive = new MecanumDrive(hardware, startPose, robotImpl::getVoltage);
                    return mecanumDrive.actionBuilder(startPose);
                },
                new RobotImplThread(hardware)
        );
    }

    @Override
    public String optionDriverUpdateCallback() {
        builder = splitAuto.fillBuilder(delay, startPos, super.getAlliance(), score, collectCount, park, gate);
        path = builder.build();

        return "\nDone building paths";
    }

    private boolean runAgain = true;
    private FtcDashboard dash = null;
    private Canvas canvas = null;
    private Action action = null;

    @Override
    protected void start_internal() {
        super.start_internal();

        Alliance.lastAlliance = getAlliance();
        AtomicBoolean pathDone = new AtomicBoolean(false);
        action = new ParallelAction(
                new SequentialAction(
                        path,
                        new InstantAction(() -> {
                            pathDone.set(true);
                        })

                ),
                (TelemetryPacket telemetryPacket) -> {
                    hardware.voltageSensor.periodic();
                    telemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
                    telemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
                    telemetry.addData("heading (deg)", Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
                    telemetry.update();
                    return !pathDone.get();
                }
        );

        //This is stolen from Actions.runBlocking(), but it is put here in a way that we can use our loop time calculations
        dash = FtcDashboard.getInstance();
        canvas = new Canvas();
        action.preview(canvas);

        runAgain = true;
    }

    @Override
    protected void loop_internal() {
        hardware.modules.parentModule.module.clearBulkCache();

        if (!runAgain || Thread.currentThread().isInterrupted()) {
            this.requestOpModeStop();
        } else {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());
            runAgain = action.run(packet);
            dash.sendTelemetryPacket(packet);
        }

        super.loop_internal();
    }
}
