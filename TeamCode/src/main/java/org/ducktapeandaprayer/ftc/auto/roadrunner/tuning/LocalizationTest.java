package org.ducktapeandaprayer.ftc.auto.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ducktapeandaprayer.ftc.auto.roadrunner.Drawing;
import org.ducktapeandaprayer.ftc.auto.roadrunner.MecanumDrive;
import org.ducktapeandaprayer.ftc.auto.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LocalizationTest extends LinearOpMode {
    private int par0InitalTicks = 0;
    private int par1InitalTicks = 0;
    private int perpInitalTicks = 0;
    private double IMUInital = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ThreeDeadWheelLocalizer localizer = (ThreeDeadWheelLocalizer) drive.localizer;

        waitForStart();
        par0InitalTicks = localizer.par0.getPositionAndVelocity().position;
        par1InitalTicks = localizer.par1.getPositionAndVelocity().position;
        perpInitalTicks = localizer.perp.getPositionAndVelocity().position;
        IMUInital = drive.hardware.getContainer().lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();


            Pose2d pose = localizer.getPose();

            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("Localizer heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Localizer heading velocity (deg / sec)", Math.toDegrees(localizer.getPoseVelocity().angVel));
            telemetry.addData("par0 ticks", localizer.par0.getPositionAndVelocity().position - par0InitalTicks);
            telemetry.addData("par1 ticks", localizer.par1.getPositionAndVelocity().position - par1InitalTicks);
            telemetry.addData("perp ticks", localizer.perp.getPositionAndVelocity().position - perpInitalTicks);
            telemetry.addData("IMU Heading (deg)", drive.hardware.getContainer().lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - IMUInital);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
