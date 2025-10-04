package net.polarbub.ftc.auto.roadrunner.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import net.polarbub.ftc.auto.roadrunner.MecanumDrive;

public final class SplineTest extends LinearOpMode {
    public static double DIAMETER = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        double radius = DIAMETER/2;

        while(opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(radius, radius), Math.PI / 2)
                            .splineTo(new Vector2d(0, DIAMETER), Math.PI)
                            .setReversed(true)
                            .splineTo(new Vector2d(radius, radius), -Math.PI / 2)
                            .splineTo(new Vector2d(0, 0), Math.PI)
                            .build());
            telemetry.update();
        }

    }
}
