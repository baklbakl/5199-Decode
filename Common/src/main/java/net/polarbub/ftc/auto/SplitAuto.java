package net.polarbub.ftc.auto;

import com.acmerobotics.roadrunner.*;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.auto.options.*;
import org.apache.commons.lang3.function.TriConsumer;

import java.lang.Math;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import static net.polarbub.ftc.MainBotConfig.ArmConfig;

public class SplitAuto {
    private static final double parkX = 35;
    private static final double parkY = -70 + (MainBotConfig.robotChassisLength / 2);

    public static int pickupPivotCounts = -290;
    public static int pickupSlideCounts = 420;

    public static int highHoverPivotCounts = -1300;
    public static int extraHighHoverPivotCounts = -1600;
    public static int slidesDoneCounts = 1400;

    public static double chamberX = -2.5;
    public static double chamberY = -36;
    public static double chamberApproachSpeed = 7;
    public static double chamberLineUpDistance = 7;
    public static double chamberDistanceBetween = 4;
    public static int chamberPullDownTime = 600;
    public static int chamberExpelTime = 200;
    public static double chamberMoveExpelTime = 1;

    public static double spikeUpX = 38.5;
    public static double spikeY = -25;
    public static double spikeTurnMaxSpeed = 15;

    public static double specimenIntakeX = 48;
    public static double specimenIntakeY = -60;
    public static double specimenLineUpDistance = 9;
    public static double specimenApproachSpeed = 15;
    public static double specimenIntakeSpeed = 7;
    public static double specimenIntakeUpPivotDelay = 0.200;
//    public static int specimenExpelTime = 150;
    //Sensor mode
    public static int specimenExpelTime = 50;

    public static double sampleIntakeX = -49;
    public static double sampleIntakeY = -41;
    public static double distanceBetweenSpike = 12;
    public static double sampleLineUpDistance = 3;
    public static double sampleIntakeSpeed = 7;

    public static double wallSpikeIntakeX = -58;
    public static double wallSpikeIntakeY = -32;
    public static double wallSpikeIntakeAngle = Math.toRadians(160);
    public static double wallSpikeApproachSpeed = 20;
    public static double wallSpikeLineUpDistance = 1;
    public static double wallSpikeIntakeTime = 0.2;

    public static double basketX = -56;
    public static double basketY = -58;
    public static double basketLineUpDistance = 6;
    public static double basketApproachSpeed = 20;
    public static int basketDropTime = 1200;

    public static int runTimes = 10;
    public static int distSensorPollInterval = 50;
    public static int distSensorStartWait = 100;

    private final Consumer<Double> setServoPosition;
    private final Consumer<Motor> homeMotor;
    private final Function<Pose2d, TrajectoryActionBuilder> createPathBuilder;

    public interface MotorFunction extends BiConsumer<Motor, Integer> {}
    private final MotorFunction runMotorToPos;
    private final MotorFunction runMotorToPosBlocking;
    private final TriConsumer<Motor, Integer, Integer> runMotorToPosBlockingEnd;
    private final Supplier<Double> getColorDist;

    private boolean stopThread = false;
    private Thread distSensorThread = null;
    private Vector2d basketLineUpPos;
    private Vector2d basketPos;
    private Vector2d sampleIntakePos;
    private Vector2d wallSampleIntakePos;
    private Vector2d specimenIntakePos;
    private Vector2d chamberPos;
    private boolean useDistSensor = true;

    public SplitAuto(Function<Pose2d, TrajectoryActionBuilder> createPathBuilder, Consumer<Double> setServoPosition, MotorFunction runMotorToPos, MotorFunction runMotorToPosBlocking, TriConsumer<Motor, Integer, Integer> runMotorToPosBlockingEnd, Consumer<Motor> homeMotor, Supplier<Double> getColorDist) {
        this.setServoPosition = setServoPosition;
        this.runMotorToPos = runMotorToPos;
        this.runMotorToPosBlocking = runMotorToPosBlocking;
        this.runMotorToPosBlockingEnd = runMotorToPosBlockingEnd;
        this.homeMotor = homeMotor;
        this.createPathBuilder = createPathBuilder;
        this.getColorDist = getColorDist;
    }

    private void runSlideAndPivot(int pivotCounts, int slideCounts) {
        runMotorToPos.accept(Motor.SLIDE, slideCounts);
        runMotorToPosBlocking.accept(Motor.PIVOT, pivotCounts);
        runMotorToPosBlocking.accept(Motor.SLIDE, slideCounts);
    }
    
    private void startDistSensor() {
        if(useDistSensor) {
            stopThread = false;
            distSensorThread = new Thread(() -> {
                try {
                    Thread.sleep(distSensorStartWait);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                while(!stopThread) {
                    if(getColorDist.get() < MainBotConfig.distSensorThresh) {
                        setServoPosition.accept(ArmConfig.restingServoPos);
                    }
                    try {
                        Thread.sleep(distSensorPollInterval);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            });
            distSensorThread.start();
        }
    }

    private void stopDistSensorBlocking() {
        if(useDistSensor) {
            stopThread = true;
            try {
                distSensorThread.join();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private TrajectoryActionBuilder dropInBasket(TrajectoryActionBuilder pathBuilder) {
        return pathBuilder
                .stopAndAdd(() -> {
//                    runMotorToPosBlocking.accept(Motor.PIVOT, ArmConfig.closeIntakePivotCounts);
                    runMotorToPosBlockingEnd.accept(Motor.PIVOT, MainBotConfig.ArmConfig.basketPivotCounts, extraHighHoverPivotCounts);
                    runMotorToPosBlockingEnd.accept(Motor.SLIDE, ArmConfig.basketSlideCounts, slidesDoneCounts);
//                    runMotorToPosBlockingEnd.accept(Motor.SLIDE, ArmConfig.closeIntakeSlideCounts, 1500);
                })
                .setReversed(true)
                .splineToConstantHeading(basketPos, Math.toRadians(45), new TranslationalVelConstraint(basketApproachSpeed))
                .stopAndAdd(() -> {
                    runMotorToPosBlocking.accept(Motor.PIVOT, MainBotConfig.ArmConfig.basketPivotCounts);
                    runMotorToPosBlocking.accept(Motor.SLIDE, ArmConfig.basketSlideCounts);
                    setServoPosition.accept(MainBotConfig.ArmConfig.dropServoPos);
                    try {
                        Thread.sleep(basketDropTime);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setServoPosition.accept(MainBotConfig.ArmConfig.restingServoPos);
                })
                .setReversed(false)
                .splineToConstantHeading(basketLineUpPos, Math.toRadians(45));
    }

    private TrajectoryActionBuilder intakeSample(TrajectoryActionBuilder pathBuilder, int spike) {
        double tangent;
        if(spike == 0) {
            tangent = Math.toRadians(90);
        } else {
            tangent = Math.toRadians(135);
        }

        double realSampleIntakeX = sampleIntakePos.x - (spike * distanceBetweenSpike);
        double lineUpY = sampleIntakePos.y - basketLineUpDistance;
        return pathBuilder
                .stopAndAdd(() -> {
                    runMotorToPosBlockingEnd.accept(Motor.SLIDE, ArmConfig.closeIntakeSlideCounts, slidesDoneCounts);
                    runMotorToPosBlockingEnd.accept(Motor.PIVOT, ArmConfig.closeIntakePivotCounts, highHoverPivotCounts);
                    setServoPosition.accept(ArmConfig.grabServoPos);
                    startDistSensor();

//                    runMotorToPos.accept(Motor.PIVOT, ArmConfig.closeIntakePivotCounts);
                })
                .setTangent(tangent)
                .splineToSplineHeading(new Pose2d(realSampleIntakeX, lineUpY, Math.toRadians(90)), Math.toRadians(90))
                .splineTo(new Vector2d(realSampleIntakeX, sampleIntakePos.y), Math.toRadians(90), new TranslationalVelConstraint(sampleIntakeSpeed))
                .stopAndAdd(() -> {
                    stopDistSensorBlocking();
                    setServoPosition.accept(MainBotConfig.ArmConfig.restingServoPos);
                    runMotorToPos.accept(Motor.PIVOT, extraHighHoverPivotCounts);
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(basketLineUpPos, Math.toRadians(45)), Math.PI + tangent);
    }

    /**
     * @param place position for where to place specimen on bar. Start at 1, then continue onwards
     */
    private TrajectoryActionBuilder placeOnChamber(TrajectoryActionBuilder pathBuilder, int place, boolean comingFromObservation) {
        double realX = chamberPos.x + (place * chamberDistanceBetween);
        Vector2d lineUp = new Vector2d(realX, chamberPos.y - chamberLineUpDistance);

        if(!comingFromObservation) {
            pathBuilder = pathBuilder.setReversed(true);
        }

        pathBuilder = pathBuilder
                .afterTime(comingFromObservation ? 0 : 0.3, () -> {
                    runMotorToPos.accept(Motor.PIVOT, ArmConfig.hoverPivotCounts);
                })
                .splineToConstantHeading(lineUp, Math.toRadians(comingFromObservation ? 180 : 90))
                .afterTime(0, () -> {
//                    runMotorToPos.accept(Motor.SLIDE, ArmConfig.specimenApproachSlideCounts);
                    runMotorToPos.accept(Motor.PIVOT, ArmConfig.specimenPivotCounts);
                    runMotorToPos.accept(Motor.SLIDE, ArmConfig.specimenApproachSlideCounts);
//                    runSlideAndPivot(ArmConfig.specimenPivotCounts, ArmConfig.specimenApproachSlideCounts);
                })
                //Y was -35
                // Using "place" will allow the specimen to be placed on a different point on the bar, so they dont overlap/not hang
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(realX, chamberPos.y), Math.toRadians(90), new TranslationalVelConstraint(chamberApproachSpeed))
                .stopAndAdd(() -> {
                    runSlideAndPivot(ArmConfig.specimenPivotCounts, ArmConfig.specimenApproachSlideCounts);

                    runMotorToPosBlocking.accept(Motor.SLIDE, ArmConfig.specimenAttachSlideCounts);
                    setServoPosition.accept(MainBotConfig.ArmConfig.grabServoPos);
                    try {
                        Thread.sleep(chamberPullDownTime);

                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setServoPosition.accept(ArmConfig.expelServoPos);
                    try {
                        Thread.sleep(chamberExpelTime);

                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .afterTime(chamberMoveExpelTime, () -> {
                    setServoPosition.accept(MainBotConfig.ArmConfig.restingServoPos);
                })
                .setReversed(false)
                .splineToConstantHeading(lineUp, Math.toRadians(270));

        return pathBuilder;
    }

    private TrajectoryActionBuilder intakeSpecimen(TrajectoryActionBuilder pathBuilder, boolean getSpike) {
        final Vector2d lineUp = new Vector2d(specimenIntakePos.x, specimenIntakePos.y + specimenLineUpDistance);

        pathBuilder = pathBuilder
                .stopAndAdd(() -> {
                    runMotorToPosBlockingEnd.accept(Motor.PIVOT, ArmConfig.hoverPivotCounts, highHoverPivotCounts);
                })
                .setTangent(0);

        if(getSpike) {
            pathBuilder = pathBuilder.splineToConstantHeading(new Vector2d(28, chamberPos.y - chamberLineUpDistance), Math.toRadians(0));

            if(spikeUpX <= -24) {
                pathBuilder = pathBuilder.splineToConstantHeading(new Vector2d(spikeUpX, -24), Math.toRadians(90));
            }

            pathBuilder = pathBuilder
                    .splineToConstantHeading(new Vector2d(spikeUpX, spikeY), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d((spikeUpX + specimenIntakePos.x) / 2, spikeY + Math.abs(spikeUpX - specimenIntakePos.x)), Math.toRadians(0), new TranslationalVelConstraint(spikeTurnMaxSpeed))
                    .splineToConstantHeading(new Vector2d(specimenIntakePos.x, spikeY), Math.toRadians(270), new TranslationalVelConstraint(spikeTurnMaxSpeed))
                    .splineToConstantHeading(lineUp, Math.toRadians(270));
        } else {
            pathBuilder = pathBuilder
                    .splineToConstantHeading(new Vector2d(specimenIntakePos.x, chamberY-chamberLineUpDistance), Math.toRadians(0));
        }

        pathBuilder = pathBuilder
                .afterTime(0, () -> {
                    runMotorToPos.accept(Motor.SLIDE, pickupSlideCounts);
                    runMotorToPos.accept(Motor.PIVOT, pickupPivotCounts);
                    setServoPosition.accept(MainBotConfig.ArmConfig.grabServoPos);
                    startDistSensor();
                });

        if(!getSpike) {
            pathBuilder = pathBuilder
                    .setTangent(Math.toRadians(270))
                    .splineToConstantHeading(lineUp, Math.toRadians(270));
        }

        return pathBuilder
                .splineToConstantHeading(specimenIntakePos, Math.toRadians(270), new TranslationalVelConstraint(specimenIntakeSpeed))
                .stopAndAdd(() -> {
                    stopDistSensorBlocking();
                    setServoPosition.accept(ArmConfig.expelServoPos);
                    try {
                        Thread.sleep(specimenExpelTime);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    setServoPosition.accept(ArmConfig.restingServoPos);
                })
                .setReversed(true)
                .afterTime(specimenIntakeUpPivotDelay, () -> {
                    runMotorToPos.accept(Motor.PIVOT, ArmConfig.hoverPivotCounts);
                })
                .splineToConstantHeading(lineUp, Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(specimenIntakePos.x+(lineUp.y - (chamberPos.y - chamberLineUpDistance)), chamberY-chamberLineUpDistance), Math.toRadians(180), new TranslationalVelConstraint(specimenApproachSpeed));
    }

    public TrajectoryActionBuilder fillBuilder(Delay delay, StartPos startPos, Score score, Spike spike, boolean park, boolean useDistSensor) {
        this.useDistSensor = useDistSensor;
        double x = 0;
        if(startPos == StartPos.LEFT) {
            x = -24 - (MainBotConfig.robotChassisWidth / 2);
        } else if(startPos == StartPos.CENTER) {
            x = -(MainBotConfig.robotChassisWidth / 2);
        } else if(startPos == StartPos.RIGHT) {
            x = 24 - (MainBotConfig.robotChassisWidth / 2);
        }

        double startHeading = 0;
        if(score == Score.CHAMBER) {
            startHeading = 270;
        } else if(score == Score.BASKET) {
            startHeading = 90;
        }
        Pose2d autoStartPosition = new Pose2d(x, -72 + (MainBotConfig.robotChassisLength / 2), Math.toRadians(startHeading));
        TrajectoryActionBuilder pathBuilder = createPathBuilder.apply(autoStartPosition);

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

        if(score == Score.BASKET) {
            basketPos = new Vector2d(basketX, basketY);
            sampleIntakePos = new Vector2d(sampleIntakeX, sampleIntakeY);
            basketLineUpPos = basketPos.plus(new Vector2d(sampleLineUpDistance,sampleLineUpDistance));
            wallSampleIntakePos = new Vector2d(wallSpikeIntakeX, wallSpikeIntakeY);

            pathBuilder = pathBuilder
                    .stopAndAdd(() -> {
                        runMotorToPosBlockingEnd.accept(Motor.PIVOT, extraHighHoverPivotCounts, ArmConfig.closeIntakePivotCounts);
                        runMotorToPos.accept(Motor.SLIDE, ArmConfig.closeIntakeSlideCounts);
                    })
                    .setTangent(135)
                    .splineToLinearHeading(new Pose2d(basketLineUpPos, Math.toRadians(45)), Math.toRadians(180));
            pathBuilder = dropInBasket(pathBuilder);

            if(spike.toInt() >= 1) {
                pathBuilder = intakeSample(pathBuilder, 0);
                pathBuilder = dropInBasket(pathBuilder);

                if(spike.toInt() >= 2) {
                    pathBuilder = intakeSample(pathBuilder, 1);
                    pathBuilder = dropInBasket(pathBuilder);

                    if(spike == Spike.THREE) {
//                        double lineUpY = wallSampleIntakeY - wallSampleLineUpDistance;
                        Vector2d lineUp = wallSampleIntakePos.plus(new Vector2d(wallSpikeLineUpDistance * Math.cos(Math.PI - wallSpikeIntakeAngle), -wallSpikeLineUpDistance * Math.sin(Math.PI - wallSpikeIntakeAngle)));

                        pathBuilder = pathBuilder
                                .stopAndAdd(() -> {
                                    runSlideAndPivot(highHoverPivotCounts, ArmConfig.closeIntakeSlideCounts);
                                    setServoPosition.accept(ArmConfig.grabServoPos);
                                    runMotorToPos.accept(Motor.PIVOT, ArmConfig.closeIntakePivotCounts);
                                })
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(lineUp, wallSpikeIntakeAngle), wallSpikeIntakeAngle)
                                .splineTo(wallSampleIntakePos, wallSpikeIntakeAngle, new TranslationalVelConstraint(wallSpikeApproachSpeed))
                                .afterTime(wallSpikeIntakeTime, () -> {
                                    setServoPosition.accept(ArmConfig.restingServoPos);
                                    runMotorToPos.accept(Motor.PIVOT, highHoverPivotCounts);
                                })
                                .setReversed(true)
                                .splineTo(lineUp, Math.PI + wallSpikeIntakeAngle)
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(basketLineUpPos, Math.toRadians(45)), Math.toRadians(270));
                        pathBuilder = dropInBasket(pathBuilder);
                    }
                }
            }

            if(park) {
                pathBuilder = pathBuilder
                        .stopAndAdd(() -> {
                            runMotorToPosBlockingEnd.accept(Motor.SLIDE, ArmConfig.restingSlideCounts, slidesDoneCounts);
                            runMotorToPosBlocking.accept(Motor.PIVOT, highHoverPivotCounts);
                            //highHoverPivotCounts
                        })

                        //TEST: Which basket park option is faster
                        .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-19, 0, Math.toRadians(180)), Math.toRadians(0))
                        .stopAndAdd(() -> {
                            runMotorToPosBlocking.accept(Motor.PIVOT, MainBotConfig.ArmConfig.hangPivotCounts);
                        });

//                        .splineToLinearHeading(new Pose2d(-36, -24, Math.toRadians(270)), Math.toRadians(90))
//                        .stopAndAdd(() -> {
//                            runMotorToPos.accept(Motor.PIVOT, MainBotConfig.ArmConfig.hangPivotCounts);
//                        })
//                        .splineTo(new Vector2d(-19, 0), Math.toRadians(0));

            } else {
                pathBuilder = pathBuilder.stopAndAdd(() -> {
                    runMotorToPosBlocking.accept(Motor.SLIDE, ArmConfig.closeIntakeSlideCounts);
                    runMotorToPos.accept(Motor.SLIDE, ArmConfig.restingSlideCounts);
                    runMotorToPosBlocking.accept(Motor.PIVOT, ArmConfig.hoverPivotCounts);
                });
            }

        } else if(score == Score.CHAMBER) {
            chamberPos = new Vector2d(chamberX, chamberY);
            specimenIntakePos = new Vector2d(specimenIntakeX, specimenIntakeY);

            pathBuilder = placeOnChamber(pathBuilder, 0, false);

            if(spike == Spike.THREE) {
                for (int i = 0; i < runTimes; i++) {
                    pathBuilder = intakeSpecimen(pathBuilder, false);
                    pathBuilder = placeOnChamber(pathBuilder, 2, true);
                }
            } else if(spike != Spike.DONT) {
                pathBuilder = intakeSpecimen(pathBuilder, spike == Spike.TWO);
                pathBuilder = placeOnChamber(pathBuilder, 1, true);

                if(spike == Spike.TWO) {
                    pathBuilder = intakeSpecimen(pathBuilder, false);
                    pathBuilder = placeOnChamber(pathBuilder, 2, true);
                }
            }

            if(park) {
                pathBuilder = pathBuilder
                        .afterTime(0, () -> {
                            runSlideAndPivot(ArmConfig.restingPivotCounts, ArmConfig.restingSlideCounts);
                        })
                        .splineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(270)), Math.toRadians(315));
            } else {
                pathBuilder = pathBuilder.stopAndAdd(() -> {
                    runSlideAndPivot(ArmConfig.restingPivotCounts, ArmConfig.restingSlideCounts);
                });
            }

        } else if(score == Score.DONT && park) {
            pathBuilder = pathBuilder
                    .splineTo(new Vector2d(x, -55), Math.toRadians(90))
                    .splineTo(new Vector2d(24, -36), Math.toRadians(0))
                    .splineTo(new Vector2d(parkX, parkY), Math.toRadians(270));
        }

        if(score != Score.BASKET || !park) {
            pathBuilder = pathBuilder
                    .stopAndAdd(() -> {
                        homeMotor.accept(Motor.PIVOT);
                    });
        }

        pathBuilder = pathBuilder.stopAndAdd(() -> {
            homeMotor.accept(Motor.SLIDE);
        });
        
        return pathBuilder;
    }
}