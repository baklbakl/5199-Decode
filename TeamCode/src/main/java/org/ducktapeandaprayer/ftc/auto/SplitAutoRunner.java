package org.ducktapeandaprayer.ftc.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.ducktapeandaprayer.ftc.auto.options.Delay;
import org.ducktapeandaprayer.ftc.auto.options.StartPos;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.auto.roadrunner.MecanumDrive;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.IntOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.lang.Math;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Split Auto")
public class SplitAutoRunner extends FastIterativeOpMode {
    private OptionDriver optionDriver;
    private MainBotHardware hardware;
    private SplitAuto splitAuto;
    private Action path = null;
    private TrajectoryActionBuilder builder = null;
    private MecanumDrive mecanumDrive;

    public Alliance alliance;
    public StartPos startPos;
    public Delay delay;
    public boolean score;
    public int spikeCount;
    public boolean park;
    public boolean gate;

    @Override
    protected void init_internal() {
        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new EnumOption<>("Alliance", "dpad_up", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
            optionDriver.add(new EnumOption<>("Start Pos", "dpad_down", gamepad1, this.getClass().getField("startPos"), this, StartPos.class));
            optionDriver.add(new EnumOption<>("Delay", "cross", gamepad1, this.getClass().getField("delay"), this, Delay.class));
            optionDriver.add(new BooleanOption("Score", "circle", gamepad1, this.getClass().getField("score"), this));
            optionDriver.add(new IntOption("Spike Count", "square", gamepad1, this.getClass().getField("spikeCount"), this, 0, 3));
            optionDriver.add(new BooleanOption("Park", "triangle", gamepad1, this.getClass().getField("park"), this));
            optionDriver.add(new BooleanOption("Gate", "dpad_left", gamepad1, this.getClass().getField("gate"), this));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        hardware = MainBotHardware.getOrCreate(hardwareMap);
        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        splitAuto = new SplitAuto(
                (startPose) -> {
                    mecanumDrive = new MecanumDrive(hardware, startPose);
                    return mecanumDrive.actionBuilder(startPose);
                },
                new RobotImpl(hardware, this::opModeIsActive)
        );
    }

    @Override
    protected void init_loop_internal() {
        String out = optionDriver.tick();
        if(out != null) {
            telemetry.addLine(out);
            telemetry.update();

            if(alliance == Alliance.BLUE) {
                hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (alliance == Alliance.RED) {
                hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            builder = splitAuto.fillBuilder(delay, startPos, alliance, score, spikeCount, park, gate);
            path = builder.build();

            telemetry.addLine(out);
            telemetry.addLine("\nDone building paths");
            telemetry.update();
        }
        try {
            Thread.sleep(1);
        } catch (InterruptedException ignored) {}
    }

    @Override
    protected void start_internal() {
        Alliance.lastAlliance = alliance;
        AtomicBoolean pathDone = new AtomicBoolean(false);
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                path,
                                new InstantAction(() -> {
                                    pathDone.set(true);
                                })

                        ),
                        (TelemetryPacket telemetryPacket) -> {
                            if(hardware.moverDrive.getPower() != 0) {
                                if(hardware.parentLauncherSwitch.getState()) {
                                    hardware.moverDrive.setPower(MainBotHardware.moverDriveAutoPower);
                                } else {
                                    hardware.moverDrive.setPower(MainBotHardware.moverDriveBallInPower);

                                }
                            }

                            telemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
                            telemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
                            telemetry.addData("heading (deg)", Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
                            telemetry.update();
                            return !pathDone.get();
                        }
                )
        );
        pathDone.set(true);
        this.requestOpModeStop();
    }

    @Override
    protected void loop_internal() {
    }


    protected void stop_internal() {
        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}
