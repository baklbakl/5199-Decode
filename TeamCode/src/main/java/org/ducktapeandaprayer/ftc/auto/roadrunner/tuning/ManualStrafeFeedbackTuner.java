package org.ducktapeandaprayer.ftc.auto.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ducktapeandaprayer.ftc.auto.roadrunner.MecanumDrive;
import org.ducktapeandaprayer.ftc.auto.roadrunner.ThreeDeadWheelLocalizer;

public final class ManualStrafeFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    //FIX: Deduplicate this with ManualFeedbackTuner
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
            throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }

        waitForStart();
        ThreeDeadWheelLocalizer localizer = (ThreeDeadWheelLocalizer) drive.localizer;

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .setTangent(Math.toRadians(90))
                            .lineToY(DISTANCE)
                            .lineToY(0)
                            .build());

            telemetry.addData("Odo0: ", localizer.par0.getPositionAndVelocity().position);
            telemetry.addData("Odo1: ", localizer.par1.getPositionAndVelocity().position);
            telemetry.update();
        }

    }
}
