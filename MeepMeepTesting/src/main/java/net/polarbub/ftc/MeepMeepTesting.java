package net.polarbub.ftc;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import net.polarbub.ftc.auto.SplitAuto;
import net.polarbub.ftc.auto.options.*;
import org.jetbrains.annotations.NotNull;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

class CustomVelConstraint implements VelConstraint {
    AngularVelConstraint base = new AngularVelConstraint(Math.PI / 2);

    @Override
    public double maxRobotVel(@NotNull Pose2dDual<Arclength> robotPose, @NotNull PosePath path, double s) {
        Pose2dDual spot = path.get(s, 3);
        return base.maxRobotVel(new Pose2dDual(spot.position, spot.position.drop(1).angleCast()), path, s);

    }
}
class AroundCornerConstraint implements VelConstraint {
    @Override
    public double maxRobotVel(@NotNull Pose2dDual<Arclength> robotPose, @NotNull PosePath path, double s) {
        {
            double radius = 0;
            double magnitudeSquared = Math.pow(robotPose.position.x.get(2), 2) +  Math.pow(robotPose.position.y.get(2), 2);

            double accel = magnitudeSquared / radius;
        }

        {
            double radius = 0;
            double maxAccel = 30;
            double maxVel = Math.sqrt(maxAccel * radius);
        }


        return 10;
    }
}

public class MeepMeepTesting {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, 1.4, 1.5, 12.5)
                .setDimensions(MainBotConfig.robotChassisWidth, MainBotConfig.robotChassisLength)
//                .setConstraints(30.0, 60.0, Math.toRadians(60.0), Math.toRadians(60.0), 15.0)
                .build();

        TrajectoryActionBuilder builder = bot.getDrive().actionBuilder(new Pose2d(0,0,0)).fresh();

        TrajectoryActionBuilder newBuilder = new TrajectoryActionBuilder(
                builder.getTurnActionFactory(),
                builder.getTrajectoryActionFactory(),
                builder.getTrajectoryBuilderParams(),
                new Pose2d(0,0,0),
                0.0,
                builder.getBaseTurnConstraints(),
                new MinVelConstraint(
                        List.of(
                                new TranslationalVelConstraint(40.0),
                                new CustomVelConstraint()
                        )
                ),
                builder.getBaseAccelConstraint()
            );

        bot.runAction(
//                bot.getDrive().actionBuilder(new Pose2d(0,0,0))
//                .turn(Math.toRadians(720))
//                .build()

//                KotlinTestKt.fillBuilder(bot.getDrive().actionBuilder(new Pose2d(-30, 0, 0)), new Pose2d(-30, 0, 0))
//                        .splineToConstantHeading(new Vector2d(10, 0), Math.toRadians(0))
//                        .splineToConstantHeading(new Vector2d(15, 5), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(10, 10), Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(-30, 10), Math.toRadians(180))
//                        .build()

                new SplitAuto(
                        (startPose) -> bot.getDrive().actionBuilder(startPose),
                        (doubl) -> {},
                        (motor, pos) -> {},
                        (motor, pos) -> {},
                        (motor, pos, endpos) -> {}, (motor) -> {},
                        () -> { return 0.0; }
                ).fillBuilder(
                    Delay.NONE,
                    StartPos.CENTER,
                    Score.CHAMBER   ,
                    Spike.TWO,
                    false,
                    false
                ).build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
