package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
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
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "PID To Point Auto")
public class PidToPointAuto extends ThreadedOpMode {
    public static double gateIntakeTimeout = 1.5;
    public static double gateWaitTime = 0.5;

    private static final Pose2d startPose = new Pose2d(-71 + 11, 71 - 31, Math.toRadians(0));
    private static final Pose2d closeShootPose = new Pose2d(-22, 22, Math.toRadians(45));
    private static final Pose2d lastShootPose = new Pose2d(-42, 16, Math.toRadians(45));

    private static final double spikeOneX = -10;
    private static final double spikeTwoX = 10;
    private static final double spikeStartY = 26;
    private static final double spikeEndY = 48;

    private static final Pose2d spikeOneLineupPose = new Pose2d(spikeOneX - 5, spikeStartY, Math.toRadians(90));
    private static final Pose2d spikeOneDonePose = new Pose2d(spikeOneX, spikeEndY + 3, Math.toRadians(90));
    private static final Pose2d spikeTwoLineupPose = new Pose2d(spikeTwoX - 12, spikeStartY, Math.toRadians(90));
    private static final Pose2d spikeTwoDonePose = new Pose2d(spikeTwoX, spikeEndY, Math.toRadians(90));

    private static final Pose2d gateHitPose = new Pose2d(6, 53, Math.toRadians(90));
    private static final Pose2d gateHitBackupPose = new Pose2d(3, spikeStartY, Math.toRadians(90));

    private static final Pose2d gatePickupLineupPose = new Pose2d(1, 39, Math.toRadians(45));
    private static final Pose2d gatePickupPreTurnPose = new Pose2d(7, 49, Math.toRadians(90));
    private static final Pose2d gatePickupPose = new Pose2d(12, 59.5, Math.toRadians(125));

    private final AtomicReference<Pose2d> lastPose = new AtomicReference<>(null);
    private OctoQuad.LocalizerDataBlock lastBlock = null;

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
        actionFactory = new FollowerActionFactory(follower, () -> lastBlock, this::invertIfBlue);

        robotImpl = new RobotImplThread(hardware, this, lastPose::get);
        super.getThreads().put("RobotImplThread", robotImpl);

        //ADD: Report beam break state in init to make sure beam breaks are broken

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
                actionFactory.makeFollowerAction(List.of(gateHitBackupPose, closeShootPose)),
                launch()
        );
    }

    private Action gateCycle(Pose2d launchPose) {
        return new SequentialAction(
                new InstantAction(robotImpl::intake),
                actionFactory.makeFollowerAction(List.of(gatePickupLineupPose, gatePickupPreTurnPose)),
                actionFactory.makeFollowerAction(gatePickupPose),
                new WaitUntilAction(robotImpl::isFull, gateIntakeTimeout),
                new InstantAction(robotImpl::stopIntake),
                actionFactory.makeFollowerAction(List.of(gatePickupPreTurnPose, launchPose)),
                launch()
        );
    }

    private Action gateCycle() {
        return gateCycle(closeShootPose);
    }

    @Override
    public String optionDriverUpdateCallback() {
        container.setOctoQuadPose(invertIfBlue(startPose));
        action = new RaceAction(
                (telemetryPacket) -> {
                    container.modules.parentModule.module.clearBulkCache();
                    container.voltageSensor.periodic();
                    lastBlock = container.readOctoQuadLocalizerData(this::isStopRequested);
                    lastPose.set(MainBotHardware.makeOctoQuadPose(lastBlock));
                    return true;
                },
                new SequentialAction(
                        new InstantAction(robotImpl::startLauncher),
                        new InstantAction(robotImpl::intake),
                        actionFactory.makeFollowerAction(closeShootPose),
                        new InstantAction(robotImpl::stopIntake),
                        launch(),
                        lineTwoCycle(),
                        lineOneCycle(),
                        gateCycle(),
                        gateCycle(),
                        gateCycle(lastShootPose)
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
