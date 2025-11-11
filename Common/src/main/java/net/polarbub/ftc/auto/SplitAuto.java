package net.polarbub.ftc.auto;

import com.acmerobotics.roadrunner.*;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.auto.options.*;

import java.lang.Math;
import java.util.function.Function;

public class SplitAuto {

    public static int distSensorPollInterval = 50;
    public static int distSensorStartWait = 100;

    public static Pose2d launchPos = new Pose2d(-50, 35, Math.toRadians(205));
    public static Pose2d obeliskPose = new Pose2d(-48, 30, Math.toRadians(270));

    private final Function<Pose2d, TrajectoryActionBuilder> createPathBuilder;
    private final RobotInterface robot;

    public SplitAuto(Function<Pose2d, TrajectoryActionBuilder> createPathBuilder, RobotInterface robot) {
        this.createPathBuilder = createPathBuilder;
        this.robot = robot;
    }

    public TrajectoryActionBuilder fillBuilder(Delay delay, StartPos startPos) {
        Pose2d autoStartPosition;

        switch (startPos) {
            case GOAL:
                autoStartPosition = new Pose2d(-71 + 2.5 + MainBotConfig.robotChassisLength/2, 71 - 24 - MainBotConfig.robotChassisWidth / 2, Math.toRadians(180));
                break;
            case CLASSIFIER:
                autoStartPosition = new Pose2d(-71 + 20 + MainBotConfig.robotChassisWidth/2, 71 - 8.5 - MainBotConfig.robotChassisLength / 2, Math.toRadians(90));
                break;
            case AUDIENCE_IN:
                autoStartPosition = new Pose2d(71 - 1.5 - MainBotConfig.robotChassisLength/2, MainBotConfig.robotChassisWidth / 2, Math.toRadians(180));
                break;
            case AUDIENCE_OUT:
                autoStartPosition = new Pose2d(71 - 1.5 - MainBotConfig.robotChassisLength/2, 22 + MainBotConfig.robotChassisWidth / 2, Math.toRadians(180));
                break;
            default:
                throw new RuntimeException("Incomplete startPos switch");
        }

        TrajectoryActionBuilder pathBuilder = createPathBuilder.apply(autoStartPosition);

        if(delay == Delay.FIVE) {
            pathBuilder = pathBuilder.waitSeconds(5);
        } else if(delay == Delay.TEN) {
            pathBuilder = pathBuilder.waitSeconds(10);
        }

        //The spike mark is on the middle of the tile. Tiles are 23.5 in square
        //Spike 1
        double spikeX;
        if (robot.getSpike() == 1) {
            spikeX = -13;
        }
        else if (robot.getSpike() == 2)
        {
            spikeX = 10.2;
        }
        else {
            spikeX = -14;
        }

        //Spike 2
//        double spikeX = 36 - 24;
        //Spike 3
//        double spikeX = 36;


        if(startPos == StartPos.GOAL || startPos == StartPos.CLASSIFIER) {
            pathBuilder = pathBuilder.setTangent(Math.toRadians(315))
                    .splineToLinearHeading(launchPos, Math.toRadians(315));
        } else {
            pathBuilder = pathBuilder.splineToLinearHeading(launchPos, Math.toRadians(180));
        }

        pathBuilder = pathBuilder
                .stopAndAdd(() -> robot.launch(3, MainBotConfig.launch1TicksPerSecAuto))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(obeliskPose, Math.toRadians(315))
//                .setTangent(Math.toRadians(angleOfArtifactRelease))
                .afterTime(0, robot::intake)
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(spikeX, 30), Math.toRadians(90))
                .splineTo(new Vector2d(spikeX, 46), Math.toRadians(90), new TranslationalVelConstraint(5))
                .setReversed(true)
                .splineTo(new Vector2d(spikeX+10, 23), Math.toRadians(320))
                .setReversed(false)
                .afterTime(0, robot::stopIntake)
                .splineToLinearHeading(launchPos, Math.toRadians(90))
                .stopAndAdd(() -> robot.launch(3, MainBotConfig.launch2TicksPerSecAuto));


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
}