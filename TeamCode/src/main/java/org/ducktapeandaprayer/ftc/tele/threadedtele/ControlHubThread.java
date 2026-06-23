package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.auto.pidtopoint.Follower;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.TripleVariableWatcher;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static java.lang.Math.abs;

public class ControlHubThread extends ThreadEx {
    //
    // Dash Options
    //
    public static Pose2d startPose = new Pose2d(70 - 7.5, - (70 - 9.65), Math.PI);

    //
    // Global Variables
    //
    public boolean followerDebugInfo = true;

    private final MainBotHardware hardware;
    private MainBotHardware.Container container;
    private final ThreadedTele threadedTele;
    private final boolean resetOctoQuadPoseOnStart;
    private final Follower follower;

    public final AtomicReference<Pose2d> robotPose = new AtomicReference<>(null);
    private Pose2d robotPoseLocal = null;

    public enum FollowerStateRequestType {
        FOLLOW,
        CANCEL,
    }

    private class FollowerStateRequest {
        private final FollowerStateRequestType requestType;
        private final List<Pose2d> points;

        private FollowerStateRequest(FollowerStateRequestType requestType, List<Pose2d> points) {
            this.requestType = requestType;
            this.points = points;
        }
    }
    private final AtomicReference<FollowerStateRequest> followerStateRequest = new AtomicReference<>(null);

    public void requestFollowerCancel() {
        followerStateRequest.set(new FollowerStateRequest(FollowerStateRequestType.CANCEL, null));
    }

    public void requestFollowerFollow(List<Pose2d> points) {
        ArrayList<Pose2d> pointsCopy = new ArrayList<>();
        for(Pose2d pose : points) {
            pointsCopy.add(threadedTele.invertIfBlue(pose));
        }
        followerStateRequest.set(new FollowerStateRequest(FollowerStateRequestType.FOLLOW, Collections.unmodifiableList(pointsCopy)));
    }

    public boolean isFollowerRequestPending() {
        return followerStateRequest.get() != null;
    }

    private volatile boolean followerRunning;
    public boolean isFollowerRunning() {
        return followerRunning;
    }


    private void computeSetWheelSpeeds(double forward, double strafe, double turning) {
        if(threadedTele.nehemiahMode && robotPoseLocal != null) {
            Vector2d controller = new Vector2d(forward, strafe);
            Rotation2d controllerRotation = Rotation2d.fromDouble(threadedTele.getAlliance() == Alliance.BLUE ? -1 : 1 * Math.PI/2);
            Vector2d fieldDirection = controllerRotation.times(controller);
            Vector2d rotated = robotPoseLocal.heading.inverse().times(fieldDirection);
            forward = rotated.x;
            strafe = rotated.y;
        }

        double leftFrontPower  = forward - strafe + turning;
        double rightFrontPower = forward + strafe - turning;
        double leftBackPower   = forward + strafe + turning;
        double rightBackPower  = forward - strafe - turning;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.max(abs(leftBackPower), Math.max(abs(leftFrontPower), abs(rightFrontPower))), abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        container.leftFrontDrive.setPower(leftFrontPower);
        container.leftRearDrive.setPower(leftBackPower);
        container.rightFrontDrive.setPower(rightFrontPower);
        container.rightRearDrive.setPower(rightBackPower);
    }

    private void resetPose(boolean resetHeading) {
        Pose2d correctedStartPose = startPose;
        if(threadedTele.getAlliance() == Alliance.BLUE) {
            correctedStartPose = new Pose2d(new Vector2d(startPose.position.x, -startPose.position.y), startPose.heading);
        }
        float heading;
        if(resetHeading) {
            heading = (float) correctedStartPose.heading.toDouble();
        } else {
            heading = container.octoQuad.readLocalizerData().heading_rad;
        }
        container.setOctoQuadPose(correctedStartPose.position.x, correctedStartPose.position.y, heading);
    }

    public ControlHubThread(MainBotHardware hardwareArg, ThreadedTele threadedTele, boolean resetOctoQuadPoseOnStart) {
        this.hardware = hardwareArg;
        this.threadedTele = threadedTele;
        this.resetOctoQuadPoseOnStart = resetOctoQuadPoseOnStart;
        this.follower = new Follower(hardware);
        follower.setPaused(false);

        try {
            threadedTele.getOptionDriver().add(new BooleanOption("Follower Debug Info", "dpad_down", threadedTele.gamepad1, this.getClass().getField("followerDebugInfo"), this));

            //Movement
            super.variablesWatched.add(new TripleVariableWatcher<>(threadedTele.gamepad1, Gamepad.class.getDeclaredField("left_stick_x"), Gamepad.class.getDeclaredField("left_stick_y"), Gamepad.class.getDeclaredField("right_stick_x"), Float::equals,
                    (Float left_stick_x_f, Float left_stick_y_f, Float right_stick_x) -> {
                if(follower.isCanceled()) {
                    computeSetWheelSpeeds(-left_stick_y_f, -left_stick_x_f, right_stick_x);
                }
            }));

            super.variablesWatched.add(new SingleVariableWatcher<>(threadedTele.gamepad1, Gamepad.class.getField("options"), Boolean::equals, (Boolean options) -> {
                if(options) {
                    resetPose(false);
                }
            }));

            super.variablesWatched.add(new SingleVariableWatcher<>(threadedTele.gamepad1, Gamepad.class.getField("share"), Boolean::equals, (Boolean share) -> {
                if(share) {
                    resetPose(true);
                }
            }));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void run() {
        container = hardware.getContainer();
        container.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        if(resetOctoQuadPoseOnStart) {
            resetPose(true);
        }

        super.run();
    }

    public void tick() {
        OctoQuad.LocalizerDataBlock block = container.readOctoQuadLocalizerData(threadedTele::isStopRequested);
        robotPoseLocal = MainBotHardware.makeOctoQuadPose(block);

        telemetry.append("Octoquad x: ");
        telemetry.append(robotPoseLocal.position.x);
        telemetry.append("\nOctoquad y: ");
        telemetry.append(robotPoseLocal.position.y);
        telemetry.append("\nOctoquad heading: ");
        telemetry.append(robotPoseLocal.heading.toDouble());
        telemetry.append("\nOctoquad heading vel: ");
        telemetry.append(block.velHeading_radS);
        telemetry.append("\n");

        robotPose.set(robotPoseLocal);

        FollowerStateRequest followerStateRequest = this.followerStateRequest.get();
        if(followerStateRequest != null) {
            switch (followerStateRequest.requestType) {
                case FOLLOW:
                    if(followerStateRequest.points == null) {
                        throw new NullPointerException("Points cannot be null when requesting a start of the a trajectory");
                    }
                    follower.followTrajectory(followerStateRequest.points, false);
                    break;
                case CANCEL:
                    follower.cancel();
                    computeSetWheelSpeeds(-threadedTele.gamepad1.left_stick_y, threadedTele.gamepad1.left_stick_x, threadedTele.gamepad1.right_stick_x);
                    break;
            }
            this.followerStateRequest.set(null);
        }

        followerRunning = !follower.isCanceled();

        if(followerDebugInfo) {
            telemetry.append(follower.tick(block));
            telemetry.append("Follower running: ").append(followerRunning).append('\n');
        } else {
            follower.tick(block);
        }

        updateTelemetry();
        container.modules.parentModule.module.clearBulkCache();
    }
}
