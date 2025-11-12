package net.polarbub.ftc;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.entity.TrajectoryActionStub;
import net.polarbub.ftc.auto.SplitAuto;
import net.polarbub.ftc.auto.options.*;
import org.jetbrains.annotations.NotNull;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

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
                .build();

//        TrajectoryActionBuilder builder = bot.getDrive().actionBuilder(new Pose2d(0,0,0)).fresh();
//        TrajectoryActionBuilder newBuilder = new TrajectoryActionBuilder(
//                builder.getTurnActionFactory(),
//                builder.getTrajectoryActionFactory(),
//                builder.getTrajectoryBuilderParams(),
//                new Pose2d(0,0,0),
//                0.0,
//                builder.getBaseTurnConstraints(),
//                new MinVelConstraint(
//                        Arrays.asList(
//                                new TranslationalVelConstraint(40.0),
//                                new CustomVelConstraint()
//                        )
//                ),
//                builder.getBaseAccelConstraint()
//            );
//
//        TrajectoryActionBuilder builderTemp = newBuilder
//                .splineTo(new Vector2d(20, 0), Math.toRadians(0));
//
//        SequentialAction move1 = (SequentialAction) builderTemp.build();
//        TrajectoryActionStub move2 = (TrajectoryActionStub) move1.component1().get(0);
//        TimeTrajectory tt = move2.getTraj();
//        DisplacementProfile dp = tt.profile.dispProfile;
//
//        TimeTrajectory newTT = null;
//
//        int i = 0;
//        for (Double disp : dp.disps) {
//            Pose2dDual<Arclength> out = tt.path.get(disp, 0);
//            double x = out.position.x.get(0);
//            System.out.println(out.position.x.get(0));
//            if(Math.abs(x - 10.0) < 0.001) {
//                newTT = new TimeTrajectory(tt.path, new TimeProfile(new DisplacementProfile(tt.profile.dispProfile.disps.subList(0, i), tt.profile.dispProfile.vels.subList(0, i), tt.profile.dispProfile.accels.subList(0, i-1))));
//                break;
//            }
//            i++;
//        }


//        builderTemp.endTrajectory();
//
//        TrajectoryActionBuilder builderTemp2 = builderTemp.fresh()
//                .splineTo(new Vector2d(40, 20), Math.toRadians(0));
//
//        Action move2 = builderTemp2.build();



        bot.runAction(
//              new TrajectoryActionStub(newTT)


//                KotlinTestKt.fillBuilder(bot.getDrive().actionBuilder(new Pose2d(-30, 0, 0)), new Pose2d(-30, 0, 0))
//                        .splineToConstantHeading(new Vector2d(10, 0), Math.toRadians(0))
//                        .splineToConstantHeading(new Vector2d(15, 5), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(10, 10), Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(-30, 10), Math.toRadians(180))
//                        .build()

                new SplitAuto(
                        (startPose) -> bot.getDrive().actionBuilder(startPose),
                        new MeepMeepImp()
                ).fillBuilder(
                        Delay.NONE,
                        StartPos.AUDIENCE_OUT
                ).build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}
