package org.ducktapeandaprayer.ftc.auto;

import com.acmerobotics.roadrunner.*;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.auto.options.Delay;
import org.ducktapeandaprayer.ftc.auto.options.StartPos;
import org.ducktapeandaprayer.ftc.lib.Alliance;

import java.lang.Math;
import java.lang.reflect.Field;
import java.util.Collections;
import java.util.function.Function;

public class SplitAuto {
    //This is split up into its components because a pose2d isn't editable in the dashboard because its fields are final
    public static double launchPosX = -48;
    public static double launchPosY = 22;
    public static double launchPosHeading = 115;
    public static double launchVelocity = -1320;
    public static double intakeSpeed = 9;
    public static double intakeStartY = 34;
    public static double intakeEndY = 47;
    public static double[] spikeXs = new double[] {-15, 7, 29};
    public static double[] spikeXsTwo = new double[] {35, 7};
    public static double[] spikeXsGR = new double[] {-15, 7, 35, 55};


    private final Function<Pose2d, TrajectoryActionBuilder> createPathBuilder;
    private final RobotInterface robot;

    public SplitAuto(Function<Pose2d, TrajectoryActionBuilder> createPathBuilder, RobotInterface robot) {
        this.createPathBuilder = createPathBuilder;
        this.robot = robot;
    }

    public static Pose2dDual<Arclength> transformPose(Pose2dDual<Arclength> pose) {
        return new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
    }

    public TrajectoryActionBuilder fillBuilder(Delay delay, StartPos startPos, Alliance alliance, boolean score, int spikeCount) {
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

        TrajectoryActionBuilder pathBuilder = null;

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
        } else if(alliance == Alliance.RED) {
            pathBuilder = createPathBuilder.apply(autoStartPosition);
        }

        if(alliance == Alliance.BLUE) {
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
//            transformedLaunchPose = transformIfBlue(new Pose2d(launchPos.position, launchPos.heading.plus(Math.toRadians(200))));
        }

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

        Pose2d launchPos = new Pose2d(launchPosX, launchPosY, Math.toRadians(launchPosHeading));

        if(score) {
            if (startPos == StartPos.GOAL) {
                pathBuilder = pathBuilder.setTangent(Math.toRadians(315));
            } else if (startPos == StartPos.CLASSIFIER) {
                pathBuilder = pathBuilder.setTangent(Math.toRadians(270));
            }

            if (startPos == StartPos.GOAL || startPos == StartPos.CLASSIFIER) {
                pathBuilder = pathBuilder
                        .stopAndAdd(() -> robot.startLauncher(launchVelocity))
                        .splineToLinearHeading(launchPos, Math.toRadians(315));
            } else {

                pathBuilder = pathBuilder
                        .splineTo(new Vector2d(0, 20), Math.toRadians(180))
                        .stopAndAdd(() -> robot.startLauncher(launchVelocity))
                        .splineToSplineHeading(launchPos, Math.toRadians(180));
            }

            pathBuilder = pathBuilder
                    .stopAndAdd(() -> {
                        robot.launch(3, launchVelocity);
                    });

            int count = 0;
            for(double spikeX : spikeXs) {
                if(count >= spikeCount) {
                    break;
                }

                pathBuilder = pathBuilder
                        .stopAndAdd(robot::intake);

                if(count < 1) {
                    pathBuilder = pathBuilder
                            .setTangent(Math.toRadians(45));
                } else {
                    pathBuilder = pathBuilder
                            .setTangent(Math.toRadians(0));
                }

//                if(count == 1) {
//                    pathBuilder = pathBuilder
//                            .splineTo(new Vector2d(-8, 25), Math.toRadians(0));
//                }

                pathBuilder = pathBuilder
                        .splineToSplineHeading(new Pose2d(spikeX, intakeStartY, Math.toRadians(90)), Math.toRadians(90))
                        .splineTo(new Vector2d(spikeX, intakeEndY), Math.toRadians(90), new TranslationalVelConstraint(intakeSpeed))

                        .afterTime(0.3, () -> {
                            //ADD: Don't nearly stall out the mover motor. Stop it if we notice a ball is in place by adding a poll on another thread ...
                            robot.stopIntake();
                            robot.startLauncher(launchVelocity);
                        })

                        .setReversed(true)
//                .splineTo(transformIfBlue(new Vector2d(spikeX+10, 23)), transformIfBlue(Math.toRadians(320)))
//                .setReversed(false)
                        .splineToLinearHeading(launchPos, Math.toRadians(180))
                        .stopAndAdd(() -> {
                            robot.launch(3, launchVelocity);
                        });

                count++;
            }

//            pathBuilder = pathBuilder
//                    .setTangent(Math.toRadians(45))
//                    .splineToSplineHeading(new Pose2d(-13, 35, Math.toRadians(90)), Math.toRadians(90))
//                    .splineTo(new Vector2d(-13, 45), Math.toRadians(90), new TranslationalVelConstraint(10))
//
//                    .afterTime(0.3, () -> {
//                        robot.stopIntake();
//                        robot.startLauncher(launchVelocity);
//                    })
//
//                    .setReversed(true)
////                .splineTo(transformIfBlue(new Vector2d(spikeX+10, 23)), transformIfBlue(Math.toRadians(320)))
////                .setReversed(false)
//                    .splineToLinearHeading(launchPos, Math.toRadians(180))
//                    .stopAndAdd(() -> {
//                        robot.launch(3, launchVelocity);
//                        robot.intake();
//                    })
//
//                    .setTangent(Math.toRadians(45))
////                    .splineTo(new Vector2d(-8, 25), Math.toRadians(0))
//                    .splineToSplineHeading(new Pose2d(11, 37.5, Math.toRadians(90)), Math.toRadians(90))
//                    .splineTo(new Vector2d(11, 45), Math.toRadians(90), new TranslationalVelConstraint(10))
//                    .afterTime(0.3, () -> {
//                        robot.stopIntake();
//                        robot.startLauncher(launchVelocity);
//                    })
//                    .setReversed(true)
//                    .splineToLinearHeading(launchPos, Math.toRadians(180))
//                    .stopAndAdd(() -> robot.launch(3, launchVelocity));
            return pathBuilder;
        }
        else
        {
            pathBuilder = pathBuilder
                    .splineToConstantHeading(new Vector2d(71-5 - MainBotConfig.robotChassisLength / 2, 35), Math.toRadians(90));
        }

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

    public TrajectoryActionBuilder fillBuilderTwo(Delay delay, StartPos startPos, Alliance alliance, boolean score, int spikeCount) {
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

        TrajectoryActionBuilder pathBuilder = null;

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
        } else if(alliance == Alliance.RED) {
            pathBuilder = createPathBuilder.apply(autoStartPosition);
        }

        if(alliance == Alliance.BLUE) {
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
//            transformedLaunchPose = transformIfBlue(new Pose2d(launchPos.position, launchPos.heading.plus(Math.toRadians(200))));
        }

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

        Pose2d launchPos = new Pose2d(launchPosX, launchPosY, Math.toRadians(launchPosHeading));
        Pose2d launchPosTwo = new Pose2d(launchPosX-10, launchPosY, Math.toRadians(launchPosHeading));

        if(score) {
            if (startPos == StartPos.GOAL) {
                pathBuilder = pathBuilder.setTangent(Math.toRadians(315));
            } else if (startPos == StartPos.CLASSIFIER) {
                pathBuilder = pathBuilder.setTangent(Math.toRadians(270));
            }

            if (startPos == StartPos.GOAL || startPos == StartPos.CLASSIFIER) {
                pathBuilder = pathBuilder
                        .stopAndAdd(() -> robot.startLauncher(launchVelocity))
                        .splineToLinearHeading(launchPosTwo, Math.toRadians(315));
            } else {

                pathBuilder = pathBuilder
                        .splineTo(new Vector2d(0, 20), Math.toRadians(180))
                        .stopAndAdd(() -> robot.startLauncher(launchVelocity))
                        .splineToSplineHeading(launchPosTwo, Math.toRadians(180));
            }

            pathBuilder = pathBuilder
                    .stopAndAdd(() -> {
                        robot.launch(3, launchVelocity);
                    });

            int count = 0;
            for(double spikeX : spikeXsTwo) {
                if(count >= spikeCount) {
                    break;
                }

                pathBuilder = pathBuilder
                        .stopAndAdd(robot::intake);

                if(count < 1) {
                    pathBuilder = pathBuilder
                            .setTangent(Math.toRadians(45));
                } else {
                    pathBuilder = pathBuilder
                            .setTangent(Math.toRadians(0));
                }

//                if(count == 1) {
//                    pathBuilder = pathBuilder
//                            .splineTo(new Vector2d(-8, 25), Math.toRadians(0));
//                }

                pathBuilder = pathBuilder

//                        .splineToSplineHeading(new Pose2d(spikeX, intakeStartY, Math.toRadians(90)), Math.toRadians(90))
//                        .splineTo(new Vector2d(spikeX, intakeEndY), Math.toRadians(90), new TranslationalVelConstraint(intakeSpeed))

                        .afterTime(0.3, () -> {
                            //ADD: Don't nearly stall out the mover motor. Stop it if we notice a ball is in place by adding a poll on another thread ...
                            robot.stopIntake();
                            robot.startLauncher(launchVelocity);
                        })

                        .setReversed(true)
//                .splineTo(transformIfBlue(new Vector2d(spikeX+10, 23)), transformIfBlue(Math.toRadians(320)))
//                .setReversed(false)
                        .splineToLinearHeading(launchPos, Math.toRadians(180))
                        .stopAndAdd(() -> {
                            robot.launch(3, launchVelocity);
                        });

                count++;
            }

//            pathBuilder = pathBuilder
//                    .setTangent(Math.toRadians(45))
//                    .splineToSplineHeading(new Pose2d(-13, 35, Math.toRadians(90)), Math.toRadians(90))
//                    .splineTo(new Vector2d(-13, 45), Math.toRadians(90), new TranslationalVelConstraint(10))
//
//                    .afterTime(0.3, () -> {
//                        robot.stopIntake();
//                        robot.startLauncher(launchVelocity);
//                    })
//
//                    .setReversed(true)
////                .splineTo(transformIfBlue(new Vector2d(spikeX+10, 23)), transformIfBlue(Math.toRadians(320)))
////                .setReversed(false)
//                    .splineToLinearHeading(launchPos, Math.toRadians(180))
//                    .stopAndAdd(() -> {
//                        robot.launch(3, launchVelocity);
//                        robot.intake();
//                    })
//
//                    .setTangent(Math.toRadians(45))
////                    .splineTo(new Vector2d(-8, 25), Math.toRadians(0))
//                    .splineToSplineHeading(new Pose2d(11, 37.5, Math.toRadians(90)), Math.toRadians(90))
//                    .splineTo(new Vector2d(11, 45), Math.toRadians(90), new TranslationalVelConstraint(10))
//                    .afterTime(0.3, () -> {
//                        robot.stopIntake();
//                        robot.startLauncher(launchVelocity);
//                    })
//                    .setReversed(true)
//                    .splineToLinearHeading(launchPos, Math.toRadians(180))
//                    .stopAndAdd(() -> robot.launch(3, launchVelocity));
            return pathBuilder;
        }
        else
        {
            pathBuilder = pathBuilder
                    .splineToConstantHeading(new Vector2d(71-5 - MainBotConfig.robotChassisLength / 2, 35), Math.toRadians(90));
        }

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

    public TrajectoryActionBuilder fillBuilderGR(Delay delay, StartPos startPos, Alliance alliance, boolean score, int spikeCount) {
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

        TrajectoryActionBuilder pathBuilder = null;

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
        } else if(alliance == Alliance.RED) {
            pathBuilder = createPathBuilder.apply(autoStartPosition);
        }

        if(alliance == Alliance.BLUE) {
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
//            transformedLaunchPose = transformIfBlue(new Pose2d(launchPos.position, launchPos.heading.plus(Math.toRadians(200))));
        }

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

        Pose2d launchPosGR = new Pose2d(launchPosX + 20, launchPosY, Math.toRadians(launchPosHeading));

        if(score) {
            if (startPos == StartPos.GOAL) {
                pathBuilder = pathBuilder.setTangent(Math.toRadians(315));
            } else if (startPos == StartPos.CLASSIFIER) {
                pathBuilder = pathBuilder.setTangent(Math.toRadians(270));
            }

            if (startPos == StartPos.GOAL || startPos == StartPos.CLASSIFIER) {
                pathBuilder = pathBuilder
                        .stopAndAdd(() -> robot.startLauncher(launchVelocity))
                        .splineToLinearHeading(launchPosGR, Math.toRadians(315));
            } else {

                pathBuilder = pathBuilder

                        .stopAndAdd(() -> robot.startLauncher(launchVelocity))
                        .splineToSplineHeading(launchPosGR, Math.toRadians(180));
            }

            pathBuilder = pathBuilder
                    .stopAndAdd(() -> {
                        robot.launch(3, launchVelocity);
                    });

            int count = 0;
            for(double spikeX : spikeXsGR) {
//                if(count >= spikeCount) {
//                    break;
//                }

                pathBuilder = pathBuilder
                        .stopAndAdd(robot::intake);

                if(count < 1) {
                    pathBuilder = pathBuilder
                            .setTangent(Math.toRadians(45));
                } else {
                    pathBuilder = pathBuilder
                            .setTangent(Math.toRadians(0));
                }



                pathBuilder = pathBuilder


                        .splineToSplineHeading(new Pose2d(spikeX, intakeStartY, Math.toRadians(90)), Math.toRadians(90))
                        .splineTo(new Vector2d(spikeX, intakeEndY), Math.toRadians(90), new TranslationalVelConstraint(intakeSpeed));
                        if (spikeX == spikeXsGR[0]) { pathBuilder = pathBuilder.setTangent(Math.toRadians(0)).splineToSplineHeading(new Pose2d(-4, 59, Math.toRadians(90)), Math.toRadians(90));}
                        if (spikeX == spikeXsGR[3]) { pathBuilder = pathBuilder.splineToSplineHeading(new Pose2d(spikeXsGR[3], 63, Math.toRadians(0)), Math.toRadians(180)).splineTo(new Vector2d(65, 63), Math.toRadians(180)).stopAndAdd(robot::intake);}
                pathBuilder = pathBuilder
                        .afterTime(0.3, () -> {
                            //ADD: Don't nearly stall out the mover motor. Stop it if we notice a ball is in place by adding a poll on another thread ...
                            robot.stopIntake();
                            robot.startLauncher(launchVelocity);
                        })

                        .setReversed(true)

                        .splineToLinearHeading(launchPosGR, Math.toRadians(180))
                        .stopAndAdd(() -> {
                            robot.launch(3, launchVelocity);
                        });

                count++;
            }

//            pathBuilder = pathBuilder
//                    .setTangent(Math.toRadians(45))
//                    .splineToSplineHeading(new Pose2d(-13, 35, Math.toRadians(90)), Math.toRadians(90))
//                    .splineTo(new Vector2d(-13, 45), Math.toRadians(90), new TranslationalVelConstraint(10))
//
//                    .afterTime(0.3, () -> {
//                        robot.stopIntake();
//                        robot.startLauncher(launchVelocity);
//                    })
//
//                    .setReversed(true)
////                .splineTo(transformIfBlue(new Vector2d(spikeX+10, 23)), transformIfBlue(Math.toRadians(320)))
////                .setReversed(false)
//                    .splineToLinearHeading(launchPos, Math.toRadians(180))
//                    .stopAndAdd(() -> {
//                        robot.launch(3, launchVelocity);
//                        robot.intake();
//                    })
//
//                    .setTangent(Math.toRadians(45))
////                    .splineTo(new Vector2d(-8, 25), Math.toRadians(0))
//                    .splineToSplineHeading(new Pose2d(11, 37.5, Math.toRadians(90)), Math.toRadians(90))
//                    .splineTo(new Vector2d(11, 45), Math.toRadians(90), new TranslationalVelConstraint(10))
//                    .afterTime(0.3, () -> {
//                        robot.stopIntake();
//                        robot.startLauncher(launchVelocity);
//                    })
//                    .setReversed(true)
//                    .splineToLinearHeading(launchPos, Math.toRadians(180))
//                    .stopAndAdd(() -> robot.launch(3, launchVelocity));
            return pathBuilder;
        }
        else
        {
            pathBuilder = pathBuilder
                    .splineToConstantHeading(new Vector2d(71-5 - MainBotConfig.robotChassisLength / 2, 35), Math.toRadians(90));
        }



        pathBuilder = pathBuilder
                .setReversed(false)
                .splineTo(new Vector2d(-40, 39), Math.toRadians(130));

        return pathBuilder;
    }
}