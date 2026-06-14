package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.ducktapeandaprayer.ftc.auto.RobotImplThread;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ActionThreadEx;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;
import org.ducktapeandaprayer.ftc.lib.optiondriver.IntOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.lang.Math;
import java.util.List;

@Autonomous(name = "PID To Point Auto")
public class PidToPointAuto extends ThreadedOpMode {
    private final Pose2d startPose = new Pose2d(-71 + 11, 71 - 31, Math.toRadians(0));
    private final Pose2d closeShootPose = new Pose2d(-22, 22, Math.toRadians(0));
    private final Pose2d spikeTwoLineupPose = new Pose2d(-10, 28, Math.toRadians(90));
    private final Pose2d spikeTwoDonePose = new Pose2d(-10, 48, Math.toRadians(90));

    private RobotImplThread robotImpl = null;
    private MainBotHardware.Container container = null;
    private MainBotHardware hardware = null;
    private FollowerActionFactory actionFactory = null;
    private Follower follower = null;
    private Action action = null;
    public int cycleCount = 0;

    @Override
    protected void init_internal() {
        super.init_internal();

        hardware = MainBotHardware.getOrCreate(hardwareMap);
        container = hardware.getContainer();
        container.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = new Follower(hardware);
        actionFactory = new FollowerActionFactory(follower, () -> hardware.readOctoQuadLocalizerData(this::isStopRequested));

        robotImpl = new RobotImplThread(hardware);
        super.getThreads().put("RobotImplThread", robotImpl);

        try {
            OptionDriver optionDriver = getOptionDriver();
            optionDriver.add(new IntOption("Cycle Count", "dpad_down", gamepad1, this.getClass().getField("cycleCount"), this, 1, 5));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public String optionDriverUpdateCallback() {
        hardware.setOctoQuadPose(invertIfBlue(startPose));
        action = new RaceAction(
                (telemetryPacket) -> {
                    container.modules.parentModule.module.clearBulkCache();
                    container.voltageSensor.periodic();
                    return true;
                },
                new SequentialAction(
                    new InstantAction(robotImpl::startLauncher),
                    actionFactory.makeFollowerAction(closeShootPose),
                    new InstantAction(robotImpl::launch),
                    (TelemetryPacket p) -> robotImpl.launching(),
                    new InstantAction(robotImpl::intake),
                    actionFactory.makeFollowerAction(List.of(spikeTwoLineupPose, spikeTwoDonePose)),
                    new InstantAction(robotImpl::stopIntake),
                    actionFactory.makeFollowerAction(List.of(spikeTwoLineupPose, closeShootPose)),
                    new InstantAction(robotImpl::launch),
                    (TelemetryPacket p) -> robotImpl.launching()
                )
        );

        return "\nDone building paths";
    }


    @Override
    protected void start_internal() {
        Alliance.lastAlliance = getAlliance();
        super.getThreads().put("PIDToPointThread", new ActionThreadEx(action, this));
        super.start_internal();
    }
}
