package net.polarbub.ftc.auto;

import com.acmerobotics.roadrunner.*;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.auto.options.*;
import net.polarbub.ftc.lib.Alliance;

import java.lang.Math;
import java.util.function.Function;

public class SplitAuto {
    public static int distSensorPollInterval = 50;
    public static int distSensorStartWait = 100;

    public static Pose2d obeliskPose = new Pose2d(-48, 30, Math.toRadians(280));
    public static Pose2d launchPos = new Pose2d(-25, 22, Math.toRadians(132));
    public static double launch1TicksPerSecAuto = 1400;
    public static double launch1TicksPerSecAutoBlue = 1400;
    public static double launch2TicksPerSecAuto = 1400;
    public static double launch2TicksPerSecAutoBlue = 1400;

    private final Function<Pose2d, TrajectoryActionBuilder> createPathBuilder;
    private final RobotInterface robot;


    private Alliance alliance = null;

    public SplitAuto(Function<Pose2d, TrajectoryActionBuilder> createPathBuilder, RobotInterface robot) {
        this.createPathBuilder = createPathBuilder;
        this.robot = robot;
    }

    public static Pose2dDual<Arclength> transformPose(Pose2dDual<Arclength> pose) {
        return new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
    }

    public Pose2d transformIfBlue(Pose2d pose) {
        if(alliance == Alliance.BLUE) {
            return new Pose2d(pose.position.x, -pose.position.y, transformIfBlue(pose.heading.toDouble()));
        } else {
            return pose;
        }
    }

    public Vector2d transformIfBlue(Vector2d pose) {
        if(alliance == Alliance.BLUE) {
            return new Vector2d(pose.x, -pose.y);
        } else {
            return pose;
        }
    }

    public double transformIfBlue(double angle) {
        if(alliance == Alliance.BLUE) {
            return (Math.toRadians(360) - angle);
        } else {
            return angle;
        }
    }

    public TrajectoryActionBuilder fillBuilder(Delay delay, StartPos startPos, Alliance alliance, boolean useVision, boolean score) {
        this.alliance = alliance;

        Pose2d autoStartPosition;

        double audienceHeading = 0;
        if(score) {
            audienceHeading = 180;
        } else {
            audienceHeading = 90;
        }

        switch (startPos) {
            case GOAL:
                autoStartPosition = new Pose2d(-71 + 2.5 + MainBotConfig.robotChassisLength/2, 71 - 24 - MainBotConfig.robotChassisWidth / 2, Math.toRadians(180));
                break;
            case CLASSIFIER:
                autoStartPosition = new Pose2d(-71 + 20 + MainBotConfig.robotChassisWidth/2, 71 - 7.5 - MainBotConfig.robotChassisLength / 2, Math.toRadians(90));
                break;
            case AUDIENCE_IN:
                autoStartPosition = new Pose2d(71 - 1.5 - MainBotConfig.robotChassisLength / 2, MainBotConfig.robotChassisWidth / 2, Math.toRadians(audienceHeading));
                break;
            case AUDIENCE_OUT:
                autoStartPosition = new Pose2d(71 - 1.5 - MainBotConfig.robotChassisLength/2, 22 + MainBotConfig.robotChassisWidth / 2, Math.toRadians(audienceHeading));
                break;
            default:
                throw new RuntimeException("Incomplete startPos switch");
        }

        TrajectoryActionBuilder pathBuilder = createPathBuilder.apply(transformIfBlue(autoStartPosition));

        Pose2d transformedLaunchPose = transformIfBlue(launchPos);

//        if(alliance == Alliance.BLUE) {
//            try {
//                Field beginEndVelField = pathBuilder.getClass().getDeclaredField("beginEndVel");
//                beginEndVelField.setAccessible(true);
//
//                pathBuilder = new TrajectoryActionBuilder(
//                        pathBuilder.getTurnActionFactory(),
//                        pathBuilder.getTrajectoryActionFactory(),
//                        pathBuilder.getTrajectoryBuilderParams(),
//                        autoStartPosition,
//                        (Double) beginEndVelField.get(pathBuilder),
//                        pathBuilder.getBaseTurnConstraints(),
//                        pathBuilder.getBaseVelConstraint(),
//                        pathBuilder.getBaseAccelConstraint(),
//                        SplitAuto::transformPose
//                );
//            } catch (IllegalAccessException | NoSuchFieldException e) {
//                throw new RuntimeException(e);
//            }
//            transformedLaunchPose = transformIfBlue(new Pose2d(launchPos.position, launchPos.heading.plus(Math.toRadians(200))));
//        }

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }
    if(score) {
        //
        if (startPos == StartPos.GOAL) {
            pathBuilder = pathBuilder.setTangent(transformIfBlue(Math.toRadians(315)));
        } else if (startPos == StartPos.CLASSIFIER) {
            pathBuilder = pathBuilder.setTangent(transformIfBlue(Math.toRadians(270)));
        }

        if (startPos == StartPos.GOAL || startPos == StartPos.CLASSIFIER) {
            pathBuilder = pathBuilder
                    .splineToLinearHeading(transformedLaunchPose, transformIfBlue(Math.toRadians(315)));
        } else {

            pathBuilder = pathBuilder
                    .splineTo(transformIfBlue(new Vector2d(0, 20)), transformIfBlue(Math.toRadians(180)))
                    .splineToSplineHeading(transformedLaunchPose, transformIfBlue(Math.toRadians(180)));
        }

        pathBuilder = pathBuilder
                .stopAndAdd(() -> robot.launch(3, alliance == Alliance.BLUE ? launch1TicksPerSecAutoBlue : launch1TicksPerSecAuto))
                .setTangent(transformIfBlue(Math.toRadians(315)));
        if (useVision) {
            pathBuilder = pathBuilder
                    .splineToLinearHeading(transformIfBlue(obeliskPose), transformIfBlue(Math.toRadians(315)));

        }

        if (!useVision) {
            pathBuilder = fillBuilderPostVision(pathBuilder, useVision, 1, alliance);
        }
    }
    else
    {
        pathBuilder = pathBuilder
                .splineToConstantHeading(transformIfBlue(new Vector2d(71-5 - MainBotConfig.robotChassisLength / 2, 35)), transformIfBlue(Math.toRadians(90)));
    }
        //

        if(true) return pathBuilder;

        for (double x = 35.6; x >= -12; x = x -23.4) {
            pathBuilder = pathBuilder
                    .splineToLinearHeading(launchPos, launchPos.heading).setReversed(false)
                    .splineTo(new Vector2d(-40, 25), Math.toRadians(140))
                   //add launch
                    .splineTo(new Vector2d(-30.5, 41.9), Math.toRadians(0))
                    .splineTo(new Vector2d(x, 40), Math.toRadians(90))
                    //add intake
                    .splineTo(new Vector2d(x, 50), Math.toRadians(90))
                    .setReversed(true)
                    .splineTo(new Vector2d(x+10, 30), Math.toRadians(320));
        }

        pathBuilder = pathBuilder
                .setReversed(false)
                .splineTo(new Vector2d(-40, 39), Math.toRadians(130));

        return pathBuilder;
    }

//    public TrajectoryActionBuilder fillBuilderPostVision(boolean useVision, int spike) {
//        fillBuilderPostVision(createPathBuilder.apply())
//    }

    public TrajectoryActionBuilder fillBuilderPostVision(TrajectoryActionBuilder pathBuilder, boolean useVision, int spike, Alliance alliance) {
        double spikeX;
        if (spike == 1) {
            spikeX = -13;
        } else if (spike == 2) {
            spikeX = 10.2;
        } else if (spike == 3) {
            spikeX =  34;
        } else {
            throw new IllegalArgumentException("Invalid spike " + spike + " valid numbers are 1..3");
        }

        Pose2d transformedLaunchPose = launchPos;

        if(alliance == Alliance.BLUE) {
            System.out.println("Transforming blue alliance");
            transformedLaunchPose = transformIfBlue(launchPos);
        }

        pathBuilder = pathBuilder
                .afterTime(0, robot::intake)
                .setTangent(transformIfBlue(Math.toRadians(270)));

        if(useVision) {
            pathBuilder = pathBuilder
                    .splineToSplineHeading(transformIfBlue(new Pose2d(-36, 10, Math.toRadians(0))), transformIfBlue(Math.toRadians(0)))
                    .setTangent(transformIfBlue(Math.toRadians(0)))
                    .splineTo(transformIfBlue(new Vector2d(spikeX, 30)), transformIfBlue(Math.toRadians(90)));
        } else {
            pathBuilder = pathBuilder
                    .setTangent(transformIfBlue(Math.toRadians(45)))
                    .splineToSplineHeading(transformIfBlue(new Pose2d(spikeX, 35, Math.toRadians(90))), transformIfBlue(Math.toRadians(90)));
        }

//        System.out.println(Math.toDegrees(transformIfBlue(Math.toRadians(320))));

        pathBuilder = pathBuilder



                .splineTo(transformIfBlue(new Vector2d(spikeX, 45)), transformIfBlue(Math.toRadians(90)), new TranslationalVelConstraint(10))

                            .afterTime(0.3, robot::stopIntake)
                            .setReversed(true)
//                .splineTo(transformIfBlue(new Vector2d(spikeX+10, 23)), transformIfBlue(Math.toRadians(320)))
//                .setReversed(false)
                            .afterTime(0, robot::stopIntake)
                            .splineToLinearHeading(transformedLaunchPose, transformIfBlue(Math.toRadians(180)))

                            .stopAndAdd(() -> robot.launch(3, alliance == Alliance.BLUE ? launch2TicksPerSecAutoBlue : launch2TicksPerSecAuto))
                            .setReversed(true)
                .setTangent(Math.toRadians(45))
                .splineTo(transformIfBlue(new Vector2d(-8, 25)), transformIfBlue(Math.toRadians(0)))
                .splineToSplineHeading(transformIfBlue(new Pose2d(11, 37.5, Math.toRadians(90))), transformIfBlue(Math.toRadians(90)))

                            .setReversed(false)

                .splineTo(transformIfBlue(new Vector2d(11, 45)), transformIfBlue(Math.toRadians(90)), new TranslationalVelConstraint(10))
                            .afterTime(0.3, robot::stopIntake)
                            .setReversed(true)
//
                            .afterTime(0, robot::stopIntake)
                            .splineToLinearHeading(transformedLaunchPose, transformIfBlue(Math.toRadians(180)))

                .stopAndAdd(() -> robot.launch(3, alliance == Alliance.BLUE ? launch2TicksPerSecAutoBlue : launch2TicksPerSecAuto));


        return pathBuilder;
    }
}