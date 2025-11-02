package net.polarbub.ftc.auto.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import net.polarbub.ftc.auto.roadrunner.Drawing;
import net.polarbub.ftc.auto.roadrunner.MecanumDrive;
import net.polarbub.ftc.auto.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            ThreeDeadWheelLocalizer localizer = (ThreeDeadWheelLocalizer) drive.localizer;
            Pose2d pose = localizer.getPose();

            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("Localizer heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("par0 ticks", localizer.par0.getPositionAndVelocity().position);
            telemetry.addData("par1 ticks", localizer.par1.getPositionAndVelocity().position);
            telemetry.addData("perp ticks", localizer.perp.getPositionAndVelocity().position);
            telemetry.addData("IMU Heading (deg)", drive.hardware.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
