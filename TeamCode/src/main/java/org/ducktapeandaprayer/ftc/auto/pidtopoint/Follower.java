package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Follower {
    public static double kPTranslation = 0.6;
    public static double kPHeading = 10;
    public static double headingDoneThresh = 0.02;
    public static double headingCorrectionMaxPower = 0.5;
    public static double kS = 2;
    public static double doneThresh = 1;
    public static double kDfake = 0.12;
    public static double velocityPower = 1;
    public static double strafeMultiplier = 1.5;
    public static double brakeDistance = 45;

    private final MainBotHardware hardware;
    private List<Pose2d> points = null;
    private boolean pause = false;
    private int targetPoint = 0;
    private boolean holdEnd = false;

    public Follower(MainBotHardware hardware) {
        this.hardware = hardware;
    }

    public void followTrajectory(List<Pose2d> points, boolean holdEnd) {
        this.points = points;
        this.holdEnd = holdEnd;
        targetPoint = 0;
    }

    private void restDT() {
        MainBotHardware.Container container = hardware.getContainer();
        container.leftFrontDrive.setPower(0);
        container.rightFrontDrive.setPower(0);
        container.leftRearDrive.setPower(0);
        container.rightRearDrive.setPower(0);
    }

    public void resetTrajectory() {
        targetPoint = 0;
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

    public boolean isCanceled() {
        return points == null;
    }

    public String tick(OctoQuad.LocalizerDataBlock dataBlock) {
        if(points != null) {
            MainBotHardware.Container container = hardware.getContainer();
            double voltage = container.voltageSensor.getVoltage();
            Pose2d targetPose = points.get(targetPoint);
            Pose2d robotPose = MainBotHardware.makeOctoQuadPose(dataBlock);
            PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(dataBlock.velX_mmS / 25.4, dataBlock.velY_mmS / 25.4), dataBlock.velHeading_radS);
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

            double angleToTarget = (Math.atan2(poseError.position.y, poseError.position.x) + Math.PI * 2) % (Math.PI * 2);
            Rotation2d rotationToTarget = Rotation2d.fromDouble(angleToTarget);

            double translationError = poseError.position.norm();
            //Target vel is zero
            double velError = robotVelocity.linearVel.norm() - 0;
            double velAngle = (Math.atan2(robotVelocity.linearVel.y, robotVelocity.linearVel.x) + Math.PI * 2) % (Math.PI * 2);
            double velAngleError = Math.abs(velAngle - angleToTarget);
            velAngleError = velAngleError > Math.PI ? Math.PI * 2 - velAngleError : velAngleError;
            double velErrorSignToTarget = velAngleError > Math.PI / 2 ? -1 : 1;
            //double velCorrectionPower = -velErrorSignToTarget * Math.pow(velError * kDfake, velocityPower) / Math.pow(Math.abs(translationError), kDfakePPower);
            double velCorrectionPower;
            double brakeDistanceLeft = brakeDistance - translationError;
            if(brakeDistanceLeft > 0) {
                velCorrectionPower = -velErrorSignToTarget * Math.pow(velError * kDfake, velocityPower) * (brakeDistanceLeft / brakeDistance);
            } else {
                velCorrectionPower = 0;
            }

            double proportionalPower = kS + translationError * kPTranslation;
            //Vel based breaking may need to be in a different direction to pose error
            double power = proportionalPower + velCorrectionPower;

            if(Math.abs(translationError) < doneThresh) {
                targetPoint++;
                if(targetPoint >= points.size()) {
                    targetPoint--;
                    if(!holdEnd) {
                        points = null;
                        headingPower = 0;
                    }
                    power = 0;
                }
            }

            if(power == 0) {
                headingPower += Math.signum(headingPower) * kS / voltage;
                if(Math.abs(headingPower) > headingCorrectionMaxPower) {
                    headingPower = Math.signum(headingPower) * headingCorrectionMaxPower;
                }
            }

            double powerVoltageComp = power / voltage;

            double forwardPower = rotationToTarget.real * powerVoltageComp;
            double strafePower = rotationToTarget.imag * powerVoltageComp * strafeMultiplier;
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

            Telemetry dashTelem = FtcDashboard.getInstance().getTelemetry();
            dashTelem.addData("Paused", pause);
            dashTelem.addData("Target pose", targetPose);
            dashTelem.addData("Target point index", targetPoint);
            dashTelem.addData("Pose error", poseError);
            dashTelem.addData("heading error", headingError);
            dashTelem.addData("heading power", headingPower);
            dashTelem.addData("translation error", translationError);
            dashTelem.addData("vel error", velError);
            dashTelem.addData("velErrorSignToTarge", velErrorSignToTarget);
            dashTelem.addData("vel correction power", velCorrectionPower);
            dashTelem.addData("proportional power", proportionalPower);
            dashTelem.addData("translation power", power);
            dashTelem.addData("forward power", forwardPower);
            dashTelem.addData("strafe power", strafePower);
            dashTelem.addData("left front power", leftFrontPower);
            dashTelem.addData("right front power", rightFrontPower);
            dashTelem.update();

            StringBuilder telemetry = new StringBuilder();
            telemetry.append("Paused: ").append(pause).append('\n');
            telemetry.append("Target pose: ").append(targetPose).append('\n');
            telemetry.append("Target point index: ").append(targetPoint).append('\n');
            telemetry.append("Pose error: ").append(poseError).append('\n');
            telemetry.append("heading error: ").append(headingError).append('\n');
            telemetry.append("heading power: ").append(headingPower).append('\n');
            telemetry.append("translation error: ").append(translationError).append('\n');
            telemetry.append("vel error: ").append(velError).append('\n');
            telemetry.append("velErrorSignToTarget: ").append(velErrorSignToTarget).append('\n');
            telemetry.append("vel correction power: ").append(velCorrectionPower).append('\n');
            telemetry.append("proportional power: ").append(proportionalPower).append('\n');
            telemetry.append("translation power: ").append(power).append('\n');
            telemetry.append("forward power: ").append(forwardPower).append('\n');
            telemetry.append("strafe power: ").append(strafePower).append('\n');
            telemetry.append("left front power: ").append(leftFrontPower).append('\n');
            telemetry.append("right front power: ").append(rightFrontPower).append('\n');
            return telemetry.toString();
        }

        return "";
    }
}
