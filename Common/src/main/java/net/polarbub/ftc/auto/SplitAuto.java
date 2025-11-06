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

import static net.polarbub.ftc.MainBotConfig.MoveConfig;

public class SplitAuto {

    public static int distSensorPollInterval = 50;
    public static int distSensorStartWait = 100;
    public static int angleOfArtifactRelease = 135;

    private final BiConsumer<Motor, Double> runMotor;
    private final Consumer<Double> runLaunchServo;

    private final Function<Pose2d, TrajectoryActionBuilder> createPathBuilder;

    public interface MotorFunction extends BiConsumer<Motor, Integer> {}

    private boolean stopThread = false;
    private Thread distSensorThread = null;

    private boolean useDistSensor = true;

    public SplitAuto(Function<Pose2d, TrajectoryActionBuilder> createPathBuilder, BiConsumer<Motor, Double> runMotor, Consumer<Double> runMoveServo) {
        this.runMotor = runMotor;
        this.createPathBuilder = createPathBuilder;
        this.runLaunchServo = runMoveServo;
    }
    private void runLaunchServo(double pos) {
        runLaunchServo.accept(pos);
    }

    private void runLauncher(double launcherPower, boolean runServo) {
        runMotor.accept(Motor.LAUNCHER_ONE, launcherPower);
        runMotor.accept(Motor.LAUNCHER_TWO, launcherPower);
        if (runServo) {
            runLaunchServo(launcherPower);
        }
    }

    private void runLauncher(double launcherPower) {
        runLauncher(launcherPower, false);
    }

    private void runIntake(double intakePower) {
        runMotor.accept(Motor.INTAKE, intakePower);
    }
    



    public TrajectoryActionBuilder fillBuilder(Delay delay, StartPos startPos, Park park) {

        Pose2d autoStartPosition;

        if(startPos == StartPos.TAG) {
            autoStartPosition = new Pose2d(-55, -55, Math.toRadians(angleOfArtifactRelease));
        } else if (startPos == StartPos.NEXT_TAG) {
            autoStartPosition = new Pose2d(-24, -72, Math.toRadians(90));
        } else {
            autoStartPosition = new Pose2d(70, 20, Math.toRadians(180));
        }


        TrajectoryActionBuilder pathBuilder = createPathBuilder.apply(autoStartPosition);

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

//        if(startPos == StartPos.LAUNCH) {
//            pathBuilder = pathBuilder
//                    .stopAndAdd(() -> {
//                        runLauncher(MoveConfig.launchDrivePower);
//                    })
//                    //.splineTo(new Vector2d(-40, -40), Math.toRadians(angleOfArtifactRelease));
//                    .splineTo(new Vector2d(-10, -20), Math.toRadians(180))
//                    .splineToLinearHeading(new Pose2d(-40, 25, Math.toRadians(angleOfArtifactRelease)), Math.toRadians(180));
//
//
//        } else {
//            pathBuilder = pathBuilder
////                    .setReversed(true)
//                    .splineToLinearHeading(new Pose2d(-40, 25, Math.toRadians(angleOfArtifactRelease)), 1);
//        }

        pathBuilder = pathBuilder
                .stopAndAdd(() -> {
                    runIntake(MoveConfig.intakeDrivePower);
                    try {
                      Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });

            for (double x = 35.6; x >= -15; x = x -23.4)
            {
                pathBuilder = pathBuilder

                        .setReversed(false)
                        .splineTo(new Vector2d(-40, 25), Math.toRadians(130))
                        .splineTo(new Vector2d(-30.5, 41.9), Math.toRadians(0))
                        .splineTo(new Vector2d(x, 40), Math.toRadians(90))
                        .splineTo(new Vector2d(x, 50), Math.toRadians(90))
                        .setReversed(true)
                        .splineTo(new Vector2d(x+5, 30), Math.toRadians(270));





//                        .splineTo(new Vector2d(x, 30), Math.toRadians(90));






            }
//                .splineTo(new Vector2d(-40, 25), Math.toRadians(130))
//
//                .splineTo(new Vector2d(-20, 40), Math.toRadians(0))
//
//
//                .splineTo(new Vector2d(35, -35), Math.toRadians(270))
//                .splineTo(new Vector2d(35, -50), Math.toRadians(90));
//
//            // add intake here
//        pathBuilder = pathBuilder
//                .splineTo(new Vector2d(-40, 25), Math.toRadians(130))
//                // add launch here
//                .splineTo(new Vector2d(11.8, -35), Math.toRadians(270))
//                // add intake
//                .splineTo(new Vector2d(11.8, -50), Math.toRadians(90))
//                .splineTo(new Vector2d(-40, 25), Math.toRadians(130))
//                // add launch
//                .splineTo(new Vector2d(-12, -35), Math.toRadians(270))
//                //add intake
//                .splineTo(new Vector2d(-12, -50), Math.toRadians(90))
//                .splineTo(new Vector2d(-40, 25), Math.toRadians(130))
//                .stopAndAdd(() -> {
//                    //In here
//                });
//                //add launch









        // .afterTime for continuous commands


        
        return pathBuilder;
    }
}