package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Follower {
    public static double kPTranslation = 0.3;
    public static double kPHeading = 5;
    public static double headingDoneThresh = 0.02;
    public static double headingCorrectionMaxPower = 0.5;
    public static double kS = 2;
    public static double doneThresh = 1;
    public static double kDfake = -0.1;
    public static double kDfakePPower = 2;
    public static double velocityPower = 1;
    public static double strafeMultiplier = 1.5;

    private final MainBotHardware.Container container;
    private List<Pose2d> points = null;
    private boolean pause = false;
    private int targetPoint = 0;

    public Follower(MainBotHardware.Container container) {
        this.container = container;
    }

    public void followTrajectoryfollowTrajectory(List<Pose2d> points) {
        this.points = points;
        targetPoint = 0;
    }

    private void restDT() {
        container.leftFrontDrive.setPower(0);
        container.rightFrontDrive.setPower(0);
        container.leftRearDrive.setPower(0);
        container.rightRearDrive.setPower(0);
    }

    public void cancel() {
        points = null;
        restDT();
    }

    public void setPaused(boolean pause) {
        this.pause = pause;
        if(pause) {
            restDT();
        }
    }

    public boolean isPaused() {
        return pause;
    }

    public void tick(OctoQuad.LocalizerDataBlock dataBlock) {
        this.tick(dataBlock, null);
    }

    public void tick(OctoQuad.LocalizerDataBlock dataBlock, Telemetry telemetry) {
        if(points != null && !pause) {
            double voltage = container.voltageSensor.getVoltage();
            Pose2d targetPose = points.get(targetPoint);
            Pose2d robotPose = MainBotHardware.makeOctoQuadPose(dataBlock);
            PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(dataBlock.velX_mmS / 25.4, dataBlock.velX_mmS / 25.4), dataBlock.velHeading_radS);
            Pose2d poseError = targetPose.minusExp(robotPose);
            double headingError = poseError.heading.toDouble();
            double headingPower = -1 * headingError * kPHeading;
            headingPower /= voltage;
            if(Math.abs(headingPower) > headingCorrectionMaxPower) {
                headingPower = Math.signum(headingPower) * headingCorrectionMaxPower;
            }

            if(Math.abs(headingError) < headingDoneThresh) {
                headingPower = 0;
            }

            double translationError = poseError.position.norm();
            //Target vel is zero
            double velError = robotVelocity.linearVel.norm();
            double power = Math.signum(translationError) * kS + translationError * kPTranslation + kDfake * Math.pow(velError, velocityPower) / Math.pow(Math.abs(translationError), kDfakePPower);
            power /= voltage;

            if(Math.abs(translationError) < doneThresh) {
                targetPoint++;
                if(targetPoint >= points.size()) {
                    targetPoint--;
                    power = 0;
                }
            }

            Rotation2d angleToTarget = Rotation2d.fromDouble(Math.atan2(poseError.position.y, poseError.position.x));

            double forwardPower = angleToTarget.real * power;
            double strafePower = angleToTarget.imag * power * strafeMultiplier;
            double maxTranslationPower = Math.abs(forwardPower) + Math.abs(strafePower);
            double maxAllowedTranslationPower = 1 - headingPower;
            if(maxTranslationPower > maxAllowedTranslationPower) {
                forwardPower *= maxAllowedTranslationPower / maxTranslationPower;
                strafePower *= maxAllowedTranslationPower / maxTranslationPower;
            }

            double leftFrontPower  = forwardPower - strafePower + headingPower;
            double rightFrontPower = forwardPower + strafePower - headingPower;
            double leftBackPower   = forwardPower + strafePower + headingPower;
            double rightBackPower  = forwardPower - strafePower - headingPower;

            if(!pause) {
                container.leftFrontDrive.setPower(leftFrontPower);
                container.rightFrontDrive.setPower(rightFrontPower);
                container.leftRearDrive.setPower(leftBackPower);
                container.rightRearDrive.setPower(rightBackPower);
            }

            if(telemetry != null) {
                telemetry.addData("Pause", pause);
                telemetry.addData("Target pose", targetPose);
                telemetry.addData("Target point index", targetPoint);
                telemetry.addData("Pose error", poseError);
                telemetry.addData("heading error", headingError);
                telemetry.addData("heading power", headingPower);
                telemetry.addData("translation error", translationError);
                telemetry.addData("vel error", velError);
                telemetry.addData("translation power", power);
                telemetry.addData("forward power", forwardPower);
                telemetry.addData("strafe power", strafePower);
                telemetry.addData("left front power", leftFrontPower);
                telemetry.addData("right front power", rightFrontPower);
                telemetry.update();
            }
        }
    }
}
