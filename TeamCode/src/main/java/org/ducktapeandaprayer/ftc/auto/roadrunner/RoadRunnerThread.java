package org.ducktapeandaprayer.ftc.auto.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.auto.RobotImplThread;
import org.ducktapeandaprayer.ftc.auto.SplitAuto;
import org.ducktapeandaprayer.ftc.auto.options.Delay;
import org.ducktapeandaprayer.ftc.auto.options.GatePush;
import org.ducktapeandaprayer.ftc.auto.options.StartPos;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.IntOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.lang.Math;
import java.util.concurrent.atomic.AtomicBoolean;

public class RoadRunnerThread extends ThreadEx {
    private final MainBotHardware hardware;
    private final MainBotHardware.Container container;
    private final ThreadedOpMode opMode;
    private final SplitAuto splitAuto;
    private Action path = null;
    private TrajectoryActionBuilder builder = null;
    private MecanumDrive mecanumDrive;
    private boolean runAgain = true;
    private FtcDashboard dash = null;
    private Canvas canvas = null;
    private Action action = null;

    public StartPos startPos;
    public Delay delay;
    public boolean score;
    public int collectCount;
    public boolean park;
    public GatePush gate;

    public RoadRunnerThread(MainBotHardware hardware, ThreadedOpMode opMode, RobotImplThread robot) {
        this.hardware = hardware;
        this.opMode = opMode;

        this.container = this.hardware.getContainer();
        container.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        OptionDriver optionDriver = this.opMode.getOptionDriver();
        Gamepad gamepad1 = this.opMode.gamepad1;

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
                    mecanumDrive = new MecanumDrive(hardware, startPose, robot::getVoltage, opMode::isStopRequested);
                    return mecanumDrive.actionBuilder(startPose);
                },
                robot
        );
    }

    @Override
    public String optionDriverUpdateCallback() {
        builder = splitAuto.fillBuilder(delay, startPos, opMode.getAlliance(), score, collectCount, park, gate);
        path = builder.build();

        return "\nDone building paths";
    }

    @Override
    public void run() {
        Alliance.lastAlliance = opMode.getAlliance();
        AtomicBoolean pathDone = new AtomicBoolean(false);
        action = new ParallelAction(
                new SequentialAction(
                        path,
                        new InstantAction(() -> {
                            pathDone.set(true);
                            RobotLog.ii("RoadRunnerThread", "Path done!");
                        })

                ),
                (TelemetryPacket telemetryPacket) -> {
                    container.voltageSensor.periodic();
                    telemetry.append("x: ").append(mecanumDrive.localizer.getPose().position.x).append("\n");
                    telemetry.append("y: ").append(mecanumDrive.localizer.getPose().position.y).append("\n");
                    telemetry.append("heading (deg): ").append(Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble())).append("\n");
                    RobotLog.ii("RoadRunnerThread", "Other action");
                    updateTelemetry();
                    return !pathDone.get();
                }
        );

        //This is stolen from Actions.runBlocking(), but it is put here in a way that we can use our loop time calculations
        dash = FtcDashboard.getInstance();
        canvas = new Canvas();
        action.preview(canvas);
        runAgain = true;

        super.run();
    }

    @Override
    public void tick() {
        container.modules.parentModule.module.clearBulkCache();

        if (!runAgain || Thread.currentThread().isInterrupted()) {
            opMode.requestOpModeStop();
        } else {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());
            runAgain = action.run(packet);
            dash.sendTelemetryPacket(packet);
        }
    }
}
