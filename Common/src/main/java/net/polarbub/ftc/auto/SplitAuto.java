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
                        setServoPosition.accept(MoveConfig.restingServoPos);
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


    public TrajectoryActionBuilder fillBuilder(Delay delay, StartPos startPos) {

        Pose2d autoStartPosition;

        if(startPos == StartPos.TAG) {
            autoStartPosition = new Pose2d(-55, -55, Math.toRadians(45));
        } else if (startPos == StartPos.NEXT_TAG) {
            autoStartPosition = new Pose2d(-24, -72, Math.toRadians(270));
        } else {
            autoStartPosition = new Pose2d(-55, -55, Math.toRadians(45));
        }


        TrajectoryActionBuilder pathBuilder = createPathBuilder.apply(autoStartPosition);

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

        pathBuilder = pathBuilder
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-45.5, -45.5, Math.toRadians(225)), Math.toRadians(100));

        
        return pathBuilder;
    }
}