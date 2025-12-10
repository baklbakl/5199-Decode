package net.polarbub.ftc.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import net.polarbub.ftc.auto.vision.AprilTagDetectionPipeline;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.auto.options.*;
import net.polarbub.ftc.auto.roadrunner.MecanumDrive;
import net.polarbub.ftc.lib.Alliance;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import net.polarbub.ftc.lib.optiondriver.BooleanOption;
import net.polarbub.ftc.lib.optiondriver.EnumOption;
import net.polarbub.ftc.lib.optiondriver.OptionDriver;
import org.openftc.apriltag.AprilTagDetection;

import java.lang.Math;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Split Auto")
public class SplitAutoRunner extends FastIterativeOpMode {
    private OptionDriver optionDriver;
    private MainBotHardware hardware;
    private SplitAuto splitAuto;
    private Action path = null;
    private AprilTagDetectionPipeline pipeline;
    private TrajectoryActionBuilder builder = null;
    private MecanumDrive mecanumDrive;

    public Alliance alliance;
    public StartPos startPos;
    public Delay delay;
    public boolean useVision;
    public boolean score;

    @Override
    protected void init_internal() {
        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new EnumOption<>("Alliance", "dpad_up", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
            optionDriver.add(new EnumOption<>("Start Pos", "dpad_down", gamepad1, this.getClass().getField("startPos"), this, StartPos.class));
            optionDriver.add(new EnumOption<>("Delay", "cross", gamepad1, this.getClass().getField("delay"), this, Delay.class));
            optionDriver.add(new BooleanOption("Use Vision", "triangle", gamepad1, this.getClass().getField("useVision"), this));
            optionDriver.add(new BooleanOption("Score", "circle", gamepad1, this.getClass().getField("score"), this));
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
                new RobotImpl(hardware, () -> true)
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

            builder = splitAuto.fillBuilder(delay, startPos, alliance, useVision, score);
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
        if(useVision) {
            pipeline = new AprilTagDetectionPipeline(hardware);
        }

        AtomicBoolean pathDone = new AtomicBoolean(false);
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                path,
                                new InstantAction(() -> {
                                   if(useVision) {
                                       int spike = 0;

                                       if (pipeline.getLatestDetections() != null && !pipeline.getLatestDetections().isEmpty()) {
                                           for (AprilTagDetection d : pipeline.getLatestDetections()) {
                                               if(spike != 0) {
                                                   telemetry.addData("Duplicate tag", d.id);
                                                   continue;
                                               }

                                               if (d.id == 21) {
                                                   spike = 1;
                                               } else if (d.id == 22) {
                                                   spike = 2;
                                               } else if (d.id == 23) {
                                                   spike = 3;
                                               }
                                           }
                                       }

                                       if (spike == 0) {
                                           telemetry.addLine("No obelisk tag found. Going to spike 1");
                                           spike = 1;
                                       } else {
                                           telemetry.addData("Spike", spike);
                                       }

                                       Actions.runBlocking(splitAuto.fillBuilderPostVision(builder.fresh(), useVision, spike, alliance).build());
                                   }
                                }),
                                new InstantAction(() -> {
                                    pathDone.set(true);
                                })

                        ),
                        (TelemetryPacket telemetryPacket) -> {
                            telemetry.update();
                            return !pathDone.get();
                        }
                )
        );
        pathDone.set(true);
    }

    @Override
    protected void loop_internal() {
        telemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        telemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
        telemetry.update();
    }


    protected void stop_internal() {
        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}
