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
    public static double launchLineUpDist = 5;
    public static double launchPosX = -22;
    public static double launchPosY = 22;
    public static double launchPosHeading = 141;
    public static double launchPosEndX = -42;
    public static double launchPosEndY = 16;
    public static double launchPosEndHeading = 122;
    public static double launchVelocity = 1210;
    public static double intakeSpeed = 9;
    public static double intakeStartY = 31;
    public static double intakeLineUpY = 29;
    public static double intakeEndY = 46;
    public static double gateY = 51.25;
    public static double gateX = -5.5;
    public static double[] spikeXs = new double[] {-9, 10, 34};
    public static double[] spikeXsTwo = new double[] {35, 7};
    public static double[] spikeXsGR = new double[] {-15, 7, 35, 55};
    public static double goalParkX = -63;
    public static double goalParkY = 7;

    public static double backLaunchPosX = 54;
    //public static double backLaunchPosY = 12;
    public static double backLaunchPosHeading = 150;
    public static double backLaunchPullAwayDist = 5;

    public static double depotFixedX = 62;
    public static double depotFixedY = 62;
    public static double depotFixedYLineup = 58;
    public static double depotFixedHeading = 90;

    private final Function<Pose2d, TrajectoryActionBuilder> createPathBuilder;
    private final RobotInterface robot;

    public SplitAuto(Function<Pose2d, TrajectoryActionBuilder> createPathBuilder, RobotInterface robot) {
        this.createPathBuilder = createPathBuilder;
        this.robot = robot;
    }

    public static Pose2dDual<Arclength> transformPose(Pose2dDual<Arclength> pose) {
        return new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
    }

    public TrajectoryActionBuilder fillBuilder(Delay delay, StartPos startPos, Alliance alliance, boolean score, int collectCount, boolean park, boolean gate) {
        Pose2d autoStartPosition;

        switch (startPos) {
            case GOAL:
                autoStartPosition = new Pose2d(-71 + 2.5 + MainBotConfig.robotChassisLength/2, 71 - 24 - MainBotConfig.robotChassisWidth / 2, Math.toRadians(180));
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
        }

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

        if(score) {
            pathBuilder = pathBuilder.stopAndAdd(() -> robot.setLauncherSpeed(launchVelocity));

            if (startPos == StartPos.AUDIENCE) {
                //double launchPosX =
                //This makes a nice smooth circular path based on our target heading and our desired x by computing the Y value
                double backLaunchHeadingRad = Math.toRadians(backLaunchPosHeading);
                double deltaHeading = autoStartPosition.heading.toDouble() - backLaunchHeadingRad;
                double deltaX = autoStartPosition.position.x - backLaunchPosX;
                Pose2d launchPos = new Pose2d(backLaunchPosX, autoStartPosition.position.y + deltaX / Math.sin(deltaHeading) - deltaX / Math.tan(deltaHeading), backLaunchHeadingRad);

                pathBuilder = pathBuilder
                        .splineTo(launchPos.position, launchPos.heading)
                        .stopAndAdd(() -> {
                            robot.launch(3);
                        });



                if(collectCount >= 1) {
                    Rotation2d launchPosTangentHeading = launchPos.heading.plus( Math.PI / 2);

                    for(int i = (gate && collectCount >= 2) ? 2 : 1; i > 0; i--) {
                        pathBuilder = pathBuilder.setTangent(backLaunchHeadingRad - Math.toRadians(90))
                               .splineToSplineHeading(new Pose2d(58, 36, Math.toRadians(depotFixedHeading)), Math.toRadians(depotFixedHeading))
                               .afterTime(0.3, robot::intake)
                               .splineToConstantHeading(new Vector2d(depotFixedX, depotFixedYLineup), Math.toRadians(depotFixedHeading))
                               .splineTo(new Vector2d(depotFixedX, depotFixedY), Math.toRadians(depotFixedHeading), new TranslationalVelConstraint(intakeSpeed))
                               .stopAndAdd(robot::stopIntake)
                               .setReversed(true)
                               .splineToConstantHeading(new Vector2d(58, 36), Math.toRadians(depotFixedHeading + 180))
                               .splineToSplineHeading(new Pose2d(launchPos.position.minus(new Vector2d(launchPosTangentHeading.real * launchLineUpDist, launchPosTangentHeading.imag * launchLineUpDist)), launchPos.heading), launchPosTangentHeading)
                               .splineToConstantHeading(launchPos.position, launchPosTangentHeading)
                               .stopAndAdd(() -> {
                                   robot.launch(3);
                               });
                    }
                }

                if((collectCount >= 2 && !gate) || collectCount >= 3) {
                    pathBuilder = pathBuilder
                            .afterTime(0.3, robot::intake)
                            .splineTo(new Vector2d(spikeXs[2], intakeLineUpY), Math.toRadians(90))
                            .splineTo(new Vector2d(spikeXs[2], intakeStartY), Math.toRadians(90))
                            .splineTo(new Vector2d(spikeXs[2], intakeEndY), Math.toRadians(90), new TranslationalVelConstraint(intakeSpeed))
                            .setReversed(true)
                            .afterTime(0.3, robot::stopIntake)
                            .splineTo(launchPos.position, launchPos.heading.plus(Math.PI))
                            .stopAndAdd(() -> {
                                robot.launch(3);
                            });
                }

                pathBuilder = pathBuilder.splineTo(launchPos.position.plus(new Vector2d(launchPos.heading.real * backLaunchPullAwayDist, launchPos.heading.imag * backLaunchPullAwayDist)), launchPos.heading);
            } else {
                Pose2d launchPos = new Pose2d(launchPosX, launchPosY, Math.toRadians(launchPosHeading));
                Pose2d launchPosEnd = new Pose2d(launchPosEndX, launchPosEndY, Math.toRadians(launchPosEndHeading));
                Pose2d firstLaunchPos = collectCount == 0 ? launchPosEnd : launchPos;

                pathBuilder = pathBuilder
                        .setReversed(true)
                        //heading.real = cos(theta) and heading.imag = sin(theta). Control click on heading for a good comment with more info
                        .splineTo(firstLaunchPos.position.plus(new Vector2d(firstLaunchPos.heading.real * launchLineUpDist, firstLaunchPos.heading.imag * launchLineUpDist)), firstLaunchPos.heading.plus(Math.PI))
                        .splineTo(firstLaunchPos.position, firstLaunchPos.heading.plus(Math.PI))

                        .stopAndAdd(() -> {
                            robot.launch(3);
                        });

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
                            .splineTo(new Vector2d(spikeX, intakeEndY), Math.toRadians(90), new TranslationalVelConstraint(intakeSpeed));

                    if(count == 0 && gate) {
                        pathBuilder = pathBuilder.splineToConstantHeading(new Vector2d(gateX, gateY), Math.toRadians(90));
                    }

                    pathBuilder = pathBuilder
                            .afterTime(0.3, robot::stopIntake)
                            .setReversed(true);

                    if(count == (collectCount - 1)) {
                        pathBuilder = pathBuilder.splineToLinearHeading(launchPosEnd, Math.toRadians(180));
                    } else {
                        pathBuilder = pathBuilder.splineToLinearHeading(launchPos, Math.toRadians(180));
                    }

                    pathBuilder = pathBuilder
                            .stopAndAdd(() -> {
                                robot.launch(3);
                            });

                    count++;
                }
            }

            pathBuilder = pathBuilder.stopAndAdd(() -> robot.setLauncherSpeed(0));
        }

        if(park) {
            if((startPos == StartPos.AUDIENCE) && !score) {
                pathBuilder = pathBuilder
                        .splineToConstantHeading(new Vector2d(71-5 - MainBotConfig.robotChassisLength / 2, 35), Math.toRadians(90));
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
            case AUDIENCE:
                autoStartPosition = new Pose2d(71 - 1.5 - MainBotConfig.robotChassisLength / 2, MainBotConfig.robotChassisWidth / 2, Math.toRadians(audienceHeading));
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
                        .stopAndAdd(() -> robot.setLauncherSpeed(launchVelocity))
                        .splineToLinearHeading(launchPosTwo, Math.toRadians(315));
            } else {

                pathBuilder = pathBuilder
                        .splineTo(new Vector2d(0, 20), Math.toRadians(180))
                        .stopAndAdd(() -> robot.setLauncherSpeed(launchVelocity))
                        .splineToSplineHeading(launchPosTwo, Math.toRadians(180));
            }

            pathBuilder = pathBuilder
                    .stopAndAdd(() -> {
                        robot.launch(3);
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
                            robot.setLauncherSpeed(launchVelocity);
                        })

                        .setReversed(true)
//                .splineTo(transformIfBlue(new Vector2d(spikeX+10, 23)), transformIfBlue(Math.toRadians(320)))
//                .setReversed(false)
                        .splineToLinearHeading(launchPos, Math.toRadians(180))
                        .stopAndAdd(() -> {
                            robot.launch(3);
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
            case AUDIENCE:
                autoStartPosition = new Pose2d(71 - 1.5 - MainBotConfig.robotChassisLength / 2, MainBotConfig.robotChassisWidth / 2, Math.toRadians(audienceHeading));
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
                        .stopAndAdd(() -> robot.setLauncherSpeed(launchVelocity))
                        .splineToLinearHeading(launchPosGR, Math.toRadians(315));
            } else {

                pathBuilder = pathBuilder

                        .stopAndAdd(() -> robot.setLauncherSpeed(launchVelocity))
                        .splineToSplineHeading(launchPosGR, Math.toRadians(180));
            }

            pathBuilder = pathBuilder
                    .stopAndAdd(() -> {
                        robot.launch(3);
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
                            robot.setLauncherSpeed(launchVelocity);
                        })

                        .setReversed(true)

                        .splineToLinearHeading(launchPosGR, Math.toRadians(180))
                        .stopAndAdd(() -> {
                            robot.launch(3);
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