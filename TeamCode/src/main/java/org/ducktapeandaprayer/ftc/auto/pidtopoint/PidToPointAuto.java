package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.ducktapeandaprayer.ftc.auto.RobotImplThread;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.action.ActionThreadEx;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;
import org.ducktapeandaprayer.ftc.lib.action.WaitUntilAction;
import org.ducktapeandaprayer.ftc.lib.optiondriver.IntOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.lang.Math;
import java.util.List;

@Autonomous(name = "PID To Point Auto")
public class PidToPointAuto extends ThreadedOpMode {
    public static double gateIntakeTimeout = 1;
    public static double gateWaitTime = 0.5;

    private static final Pose2d startPose = new Pose2d(-71 + 11, 71 - 31, Math.toRadians(0));
    private static final Pose2d closeShootPose = new Pose2d(-22, 22, Math.toRadians(0));

    private static final double spikeOneX = -10;
    private static final double spikeTwoX = 10;
    private static final double spikeStartY = 28;
    private static final double spikeEndY = 48;

    private static final Pose2d spikeOneLineupPose = new Pose2d(spikeOneX - 9, spikeStartY, Math.toRadians(90));
    private static final Pose2d spikeOneDonePose = new Pose2d(spikeOneX, spikeEndY, Math.toRadians(90));
    private static final Pose2d spikeTwoLineupPose = new Pose2d(spikeTwoX - 12, spikeStartY, Math.toRadians(90));
    private static final Pose2d spikeTwoDonePose = new Pose2d(spikeTwoX, spikeEndY, Math.toRadians(90));

    //private static final double gateX = 6;
    private static final Pose2d gateHitPose = new Pose2d(6, 52, Math.toRadians(90));
    private static final Pose2d gatePickupBackupPose = new Pose2d(6, 48, Math.toRadians(110));
    private static final Pose2d gatePickupPose = new Pose2d(8, 54, Math.toRadians(110));
    private static final Pose2d gateLinupPose = new Pose2d(1, spikeStartY, Math.toRadians(90));

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
        actionFactory = new FollowerActionFactory(follower, () -> hardware.readOctoQuadLocalizerData(this::isStopRequested), this::invertIfBlue);

        robotImpl = new RobotImplThread(hardware, true);
        super.getThreads().put("RobotImplThread", robotImpl);

        try {
            OptionDriver optionDriver = getOptionDriver();
            optionDriver.add(new IntOption("Cycle Count", "dpad_down", gamepad1, this.getClass().getField("cycleCount"), this, 1, 5));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    private Action launch() {
        return new SequentialAction(
                new InstantAction(robotImpl::launch),
                (TelemetryPacket p) -> robotImpl.launching()
        );
    }

    private Action lineOneCycle() {
        return new SequentialAction(
                new InstantAction(robotImpl::intake),
                actionFactory.makeFollowerAction(List.of(spikeOneLineupPose, spikeOneDonePose)),
                new InstantAction(robotImpl::stopIntake),
                actionFactory.makeFollowerAction(List.of(closeShootPose)),
                launch()
        );
    }

    private Action lineTwoCycle() {
        return new SequentialAction(
                new InstantAction(robotImpl::intake),
                actionFactory.makeFollowerAction(List.of(spikeTwoLineupPose, spikeTwoDonePose)),
                new InstantAction(robotImpl::stopIntake),
                actionFactory.makeFollowerAction(gateHitPose),
                new SleepAction(gateWaitTime),
                actionFactory.makeFollowerAction(List.of(spikeTwoLineupPose, closeShootPose)),
                launch()
        );
    }

    private Action gateCycle(boolean clearSpikeOne) {
        return new SequentialAction(
                new InstantAction(robotImpl::intake),
                clearSpikeOne ? actionFactory.makeFollowerAction(List.of(gateLinupPose, gatePickupPose)) : actionFactory.makeFollowerAction(gatePickupPose),
                new WaitUntilAction(robotImpl::isFull, gateIntakeTimeout),
                new InstantAction(robotImpl::stopIntake),
                actionFactory.makeFollowerAction(List.of(gatePickupBackupPose, closeShootPose)),
                launch()
        );
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
                        launch(),
                        lineTwoCycle(),
                        gateCycle(true),
                        lineOneCycle(),
                        gateCycle(false),
                        gateCycle(false)
                        //ADD: Park on last cycle
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
