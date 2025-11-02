package net.polarbub.ftc.auto;

import com.acmerobotics.roadrunner.*;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.auto.options.*;

import java.lang.Math;
import java.util.function.Function;

public class SplitAuto {
    public static double angleOfArtifactRelease = 135;

    private final Function<Pose2d, TrajectoryActionBuilder> createPathBuilder;
    public final RobotInterface hardware;

    public SplitAuto(Function<Pose2d, TrajectoryActionBuilder> createPathBuilder, RobotInterface hardware) {
        this.createPathBuilder = createPathBuilder;
        this.hardware = hardware;
    }

    public TrajectoryActionBuilder fillBuilder(Delay delay, StartPos startPos) {
        Pose2d autoStartPosition;

        if(startPos == StartPos.TAG) {
            autoStartPosition = new Pose2d(-55, -55, Math.toRadians(angleOfArtifactRelease));
        } else if (startPos == StartPos.NEXT_TAG) {
            autoStartPosition = new Pose2d(-24, -72, Math.toRadians(90));
        } else {
            autoStartPosition = new Pose2d(70, -20, Math.toRadians(180));
        }


        TrajectoryActionBuilder pathBuilder = createPathBuilder.apply(autoStartPosition);

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

        if(startPos == StartPos.LAUNCH) {
            pathBuilder = pathBuilder
                    .stopAndAdd(() -> {
                        hardware.setLauncherSpeed(MainBotConfig.launchTicksPerSec);
                    })
                    //.splineTo(new Vector2d(-40, -40), Math.toRadians(angleOfArtifactRelease));
                    .splineTo(new Vector2d(-10, -20), Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(angleOfArtifactRelease)), Math.toRadians(180));


        } else {
            pathBuilder = pathBuilder
//                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(angleOfArtifactRelease)), 1);
        }

        pathBuilder = pathBuilder
                .stopAndAdd(() -> {
                    hardware.runIntake(IntakeDirection.IN);
                    try {
                      Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .splineTo(new Vector2d(35, -35), Math.toRadians(270))
                .splineTo(new Vector2d(35, -50), Math.toRadians(90));
            // add intake here
        pathBuilder = pathBuilder
                .splineTo(new Vector2d(-40, -40), Math.toRadians(180));
                // add launch here

        // .afterTime for continuous commands

        return pathBuilder;
    }
}