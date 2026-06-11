package org.ducktapeandaprayer.ftc.auto;

import com.acmerobotics.roadrunner.*;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.auto.options.Delay;
import org.ducktapeandaprayer.ftc.auto.options.GatePush;
import org.ducktapeandaprayer.ftc.auto.options.StartPos;
import org.ducktapeandaprayer.ftc.lib.Alliance;

import java.lang.Math;
import java.lang.reflect.Field;
import java.util.Collections;
import java.util.function.Function;

public class SplitAuto {
    //This is split up into its components because a pose2d isn't editable in the dashboard because its fields are final
    public static double launchLineUpDist = 5;
    public static double launchPosX = -22;
    public static double launchPosY = 22;
    public static double launchPosHeading = 141;
    public static double launchPosEndX = -42;
    public static double launchPosEndY = 16;
    public static double launchPosEndHeading = 122;
    public static double intakeSpeed = 10;
    public static double intakeStartY = 30;
    public static double intakeLineUpY = 28;
    public static double intakeEndY = 45;
    public static double gateY = 56;
    public static double gateOneX = -4.5;
    public static double gateTwoX = 6;
    public static double gatePushWait = 0.5;
    public static double[] spikeXs = new double[] {10, -10, 38};
    public static double goalParkX = -63;
    public static double goalParkY = 7;

    private final Function<Pose2d, TrajectoryActionBuilder> createPathBuilder;
    private final RobotInterface robot;

    public SplitAuto(Function<Pose2d, TrajectoryActionBuilder> createPathBuilder, RobotInterface robot) {
        this.createPathBuilder = createPathBuilder;
        this.robot = robot;
    }

    public static Pose2dDual<Arclength> transformPose(Pose2dDual<Arclength> pose) {
        return new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
    }

    public TrajectoryActionBuilder fillBuilder(Delay delay, StartPos startPos, Alliance alliance, boolean score, int collectCount, boolean park, GatePush gate) {
        Pose2d autoStartPosition;

        switch (startPos) {
            case GOAL:
                autoStartPosition = new Pose2d(-71 + 13, 71 - 31, Math.toRadians(180));
                break;
            case CLASSIFIER:
                autoStartPosition = new Pose2d(-71 + 20 + MainBotConfig.robotChassisWidth/2, 71 - 7.5 - MainBotConfig.robotChassisLength / 2, Math.toRadians(90));
                break;
            case AUDIENCE:
                autoStartPosition = new Pose2d(71 - 1.5 - MainBotConfig.robotChassisLength / 2, 24 - MainBotConfig.robotChassisWidth / 2, Math.toRadians(180));
                break;
            default:
                throw new RuntimeException("Incomplete startPos switch");
        }

        TrajectoryActionBuilder pathBuilder;

        if(alliance == Alliance.BLUE) {
            pathBuilder = createPathBuilder.apply(
                    //This is a mess to feed a pose 2d into a pose2ddual to send it through transform pose, but I don't care because it works
                    transformPose(
                            new Pose2dDual<>(
                                    new DualNum<>(Collections.singletonList(autoStartPosition.position.x)),
                                    new DualNum<>(Collections.singletonList(autoStartPosition.position.y)),
                                    new DualNum<>(Collections.singletonList(autoStartPosition.heading.toDouble())
                                    )
                            )
                    ).value()
            );

            try {
                Field beginEndVelField = pathBuilder.getClass().getDeclaredField("beginEndVel");
                beginEndVelField.setAccessible(true);

                pathBuilder = new TrajectoryActionBuilder(
                        pathBuilder.getTurnActionFactory(),
                        pathBuilder.getTrajectoryActionFactory(),
                        pathBuilder.getTrajectoryBuilderParams(),
                        autoStartPosition,
                        (Double) beginEndVelField.get(pathBuilder),
                        pathBuilder.getBaseTurnConstraints(),
                        pathBuilder.getBaseVelConstraint(),
                        pathBuilder.getBaseAccelConstraint(),
                        SplitAuto::transformPose
                );
            } catch (IllegalAccessException | NoSuchFieldException e) {
                throw new RuntimeException(e);
            }
        } else if(alliance == Alliance.RED) {
            pathBuilder = createPathBuilder.apply(autoStartPosition);
        } else {
            throw new RuntimeException("Screw you intellij");
        }

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

        if(score) {
            if (startPos != StartPos.AUDIENCE) {
                Pose2d launchPos = new Pose2d(launchPosX, launchPosY, Math.toRadians(launchPosHeading));
                Pose2d launchPosEnd = new Pose2d(launchPosEndX, launchPosEndY, Math.toRadians(launchPosEndHeading));
                Pose2d firstLaunchPos = collectCount == 0 ? launchPosEnd : launchPos;

                pathBuilder = pathBuilder
                        .stopAndAdd(robot::startLauncher)
                        .setReversed(true)
                        //heading.real = cos(theta) and heading.imag = sin(theta). Control click on heading for a good comment with more info
                        .splineTo(firstLaunchPos.position.plus(new Vector2d(firstLaunchPos.heading.real * launchLineUpDist, firstLaunchPos.heading.imag * launchLineUpDist)), firstLaunchPos.heading.plus(Math.PI))
                        .splineTo(firstLaunchPos.position, firstLaunchPos.heading.plus(Math.PI))
                        .stopAndAdd(robot::launch)
                        .stopAndAdd(telemetryPacket -> robot.launching());
                int count = 0;
                for(double spikeX : spikeXs) {
                    if(count >= collectCount) {
                        break;
                    }

                    pathBuilder = pathBuilder
                            .stopAndAdd(robot::intake)
                            .setTangent(Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(spikeX, intakeLineUpY, Math.toRadians(90)), Math.toRadians(90))
                            .splineTo(new Vector2d(spikeX, intakeStartY), Math.toRadians(90))
                            .splineTo(new Vector2d(spikeX, intakeEndY), Math.toRadians(90), new TranslationalVelConstraint(intakeSpeed))
                            .afterTime(0.3, robot::stopIntake);

                    if(
                            (count == 0 && (gate == GatePush.AFTER_ONE || gate == GatePush.AFTER_BOTH))
                            || (count == 1 && (gate == GatePush.AFTER_TWO || gate == GatePush.AFTER_BOTH))
                    ) {
                        pathBuilder = pathBuilder
                                .splineToConstantHeading(new Vector2d(count == 0 ? gateTwoX : gateOneX, gateY), Math.toRadians(90))
                                .waitSeconds(gatePushWait);
                        if(count == 0) {
                            pathBuilder = pathBuilder
                                    .setReversed(true)
                                    .splineToSplineHeading(new Pose2d(gateTwoX, intakeEndY, Math.toRadians(90)), Math.toRadians(270));
                        }
                    }

                    pathBuilder = pathBuilder
                            .setReversed(true);

                    if(count == (collectCount - 1)) {
                        pathBuilder = pathBuilder.splineToLinearHeading(launchPosEnd, Math.toRadians(180));
                    } else {
                        pathBuilder = pathBuilder.splineToLinearHeading(launchPos, Math.toRadians(180));
                    }

                    pathBuilder = pathBuilder
                            .stopAndAdd(robot::launch)
                            .stopAndAdd(telemetryPacket -> robot.launching());
                    count++;
                }
            }

            pathBuilder = pathBuilder.stopAndAdd(robot::stopLauncher);
        }

        if(park) {
            if((startPos == StartPos.AUDIENCE)) {
                Vector2d backParkPos = new Vector2d(71-5 - MainBotConfig.robotChassisLength / 2, 35);
                pathBuilder = pathBuilder
                        .splineToConstantHeading(backParkPos, Math.toRadians(90));

            } else {
                if(score) {
                    pathBuilder = pathBuilder
                            .setTangent(Math.toRadians(225));
                } else {
                    pathBuilder = pathBuilder
                            .setTangent(Math.toRadians(300));
                }

                pathBuilder = pathBuilder
                        .splineToLinearHeading(new Pose2d(goalParkX, goalParkY, Math.toRadians(90)), Math.toRadians(225));
            }
        }

        return pathBuilder;
    }
}