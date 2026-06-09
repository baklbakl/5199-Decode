package org.ducktapeandaprayer.ftc;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorManager;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.ducktapeandaprayer.ftc.auto.SplitAuto;
import org.ducktapeandaprayer.ftc.auto.options.Delay;
import org.ducktapeandaprayer.ftc.auto.options.GatePush;
import org.ducktapeandaprayer.ftc.auto.options.StartPos;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.jetbrains.annotations.NotNull;

import java.lang.Math;

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

// HOW TO MAKE THE LINES ALL BOLD IN MEEPMEEP:
//  1) Add breakpoint in ActionEntity.kt at line 238
//  2) Make sure it doesn't suspend and evaluates and logs the following
//   `gfx.color = Color(colorScheme.TRAJECTORY_PATH_COLOR.red, colorScheme.TRAJECTORY_PATH_COLOR.green, colorScheme.TRAJECTORY_PATH_COLOR.blue, 255)`

public class MeepMeepTesting {
    private static double toServoPos(double heading) {
        return heading * 77/29 * 15/48 / Math.toRadians(355) + 0.5;
    }

    public static void main(String[] args) {
        System.out.println(toServoPos(0));
        System.out.println(toServoPos(Math.toRadians(213)));
        System.out.println(toServoPos(-Math.PI / 2));

        if(true) return;
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);
        meepMeep.setTheme(new ColorSchemeRedLight());
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 50, 3, 3, 12.5)
                .setDimensions(MainBotConfig.robotChassisWidth, MainBotConfig.robotChassisLength)
                .build();
        RoadRunnerBotEntity botTwo = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 55, 3, 3, 12.5)
                .setDimensions(MainBotConfig.robotChassisWidth, MainBotConfig.robotChassisLength)
                .build();
        RoadRunnerBotEntity botGR = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 55, 3, 3, 12.5)
                .setDimensions(MainBotConfig.robotChassisWidth, MainBotConfig.robotChassisLength)
                .build();




        SplitAuto splitAuto = new SplitAuto((startPose) -> bot.getDrive().actionBuilder(startPose), new MeepMeepImp());
        Alliance alliance = Alliance.RED;
        TrajectoryActionBuilder builder = splitAuto.fillBuilder(Delay.NONE, StartPos.AUDIENCE, alliance, true, 3, true, GatePush.AFTER_ONE);

        Action action = builder.build();
        bot.runAction(
//              new TrajectoryActionStub(newTT)
//                KotlinTestKt.fillBuilder(bot.getDrive().actionBuilder(new Pose2d(-30, 0, 0)), new Pose2d(-30, 0, 0))
//                        .splineToConstantHeading(new Vector2d(10, 0), Math.toRadians(0))
//                        .splineToConstantHeading(new Vector2d(15, 5), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(10, 10), Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(-30, 10), Math.toRadians(180))
//                        .build()

                action


        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
