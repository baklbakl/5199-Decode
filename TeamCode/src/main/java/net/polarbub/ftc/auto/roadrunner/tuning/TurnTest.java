package net.polarbub.ftc.auto.roadrunner.tuning;

import android.util.Log;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import net.polarbub.ftc.auto.roadrunner.MecanumDrive;

public final class TurnTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        if(!opModeIsActive()) {
            return;
        }

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .turn(Math.toRadians(720))
                        .build());
        telemetry.update();
//            Log.i("CUSTOM_LOCALIZATION", "AFTER TURN ^");

    }
}
