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
    private boolean enableTelemetry = false;

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

    public void setTelemetryEnabled(boolean enabled) {
        enableTelemetry = enabled;
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
            Pose2d poseError = new Pose2d(targetPose.position.x - robotPose.position.x, targetPose.position.y - robotPose.position.y, targetPose.heading.minus(robotPose.heading));

            //Simple proportional controller for translational
            double translationError = poseError.position.norm();
            double proportionalPower = kS + translationError * kPTranslation;
            double translationalPower = proportionalPower;

            double headingToTarget = (Math.atan2(poseError.position.y, poseError.position.x) + Math.PI * 2) % (Math.PI * 2);

            PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(dataBlock.velX_mmS / 25.4, dataBlock.velY_mmS / 25.4), dataBlock.velHeading_radS);
            //Target vel is always zero for now
            double velError = robotVelocity.linearVel.norm() - 0;
            //Figure out if we should brake by opposing the proportional error or going with it. Right now the velocity
            // correction only acts in the direction of the proportional error.
            double velHeading = (Math.atan2(robotVelocity.linearVel.y, robotVelocity.linearVel.x) + Math.PI * 2) % (Math.PI * 2);
            double velHeadingError = Math.abs(velHeading - headingToTarget);
            velHeadingError = velHeadingError > Math.PI ? Math.PI * 2 - velHeadingError : velHeadingError;
            double velErrorSignToTarget = velHeadingError > Math.PI / 2 ? -1 : 1;

            //Apply breaking that starts at a set distance from the target and increases linearly as we approach it.
            double velCorrectionPower;
            double brakeDistanceLeft = brakeDistance - translationError;
            if(brakeDistanceLeft > 0) {
                velCorrectionPower = -velErrorSignToTarget * Math.pow(velError * kDfake, velocityPower) * (brakeDistanceLeft / brakeDistance);
            } else {
                velCorrectionPower = 0;
            }

            //If this is the last point in the chain turn on breaking
            if(targetPoint + 1 == points.size()) {
                translationalPower += velCorrectionPower;
            }

            //Simple kP for heading
            double headingError = poseError.heading.toDouble();
            double headingPower = -1 * headingError * kPHeading;

            if(Math.abs(headingError) < headingDoneThresh) {
                headingPower = 0;
            }

            //If our translation has finished then move onto the next point, hold the end point, or finish the path.
            // Right now we don't wait for heading to achieve its setpoint before moving on.
            if(Math.abs(translationError) < doneThresh) {
                targetPoint++;
                if(targetPoint >= points.size()) {
                    targetPoint--;
                    if(!holdEnd) {
                        points = null;
                        headingPower = 0;
                    }
                    translationalPower = 0;
                }
            }

            //Apply kS to rotation if there is no kS from translation (translation power is zero)
            if(translationalPower == 0) {
                headingPower += Math.signum(headingPower) * kS;
            }

            double voltageCompTranslationPower = translationalPower / voltage;
            //Rotate the direction of translation required to the robot's frame. Rotation2d.times() adds two rotations.
            // Rotation2d.inverse() is a unary minus, so in total we get headingToTarget - robotPose.heading;
            Rotation2d robotRotationToTarget = Rotation2d.fromDouble(headingToTarget).times(robotPose.heading.inverse());
            //Rotation2d robotRotationToTarget = Rotation2d.fromDouble(Rotation2d.fromDouble(headingToTarget).minus(robotPose.heading));

            //Find the forward and strafe components of the translation. .real and .imag are just cos() and sin().
            // Roadrunner's rotation class stores rotations as the sin() and cos() of their angle
            double forwardPower = robotRotationToTarget.real * voltageCompTranslationPower;
            double strafePower = robotRotationToTarget.imag * voltageCompTranslationPower * strafeMultiplier;
            double voltageCompHeadingPower = headingPower / voltage;

            //Cap our wheel power at one while making sure all the wheels retain their relative power levels
            double maxPower = Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(voltageCompHeadingPower);
            if(maxPower > 1) {
                forwardPower /= maxPower;
                strafePower /= maxPower;
                headingPower /= maxPower;
            }

            //Standard Mecanum kinematics
            double leftFrontPower  = forwardPower - strafePower + voltageCompHeadingPower;
            double rightFrontPower = forwardPower + strafePower - voltageCompHeadingPower;
            double leftBackPower   = forwardPower + strafePower + voltageCompHeadingPower;
            double rightBackPower  = forwardPower - strafePower - voltageCompHeadingPower;

            if(!pause) {
                container.leftFrontDrive.setPower(leftFrontPower);
                container.rightFrontDrive.setPower(rightFrontPower);
                container.leftRearDrive.setPower(leftBackPower);
                container.rightRearDrive.setPower(rightBackPower);
            }

            if(enableTelemetry) {
                Telemetry dashTelem = FtcDashboard.getInstance().getTelemetry();
                dashTelem.addData("Paused", pause);
                dashTelem.addData("Target pose", targetPose);
                dashTelem.addData("Target point index", targetPoint);
                dashTelem.addData("Robot x", robotPose.position.x);
                dashTelem.addData("Robot y", robotPose.position.y);
                dashTelem.addData("Robot heading", robotPose.heading.toDouble());
                dashTelem.addData("Robot RotationToTarget", robotRotationToTarget.toDouble());
                dashTelem.addData("Pose error x", poseError.position.x);
                dashTelem.addData("Pose error y", poseError.position.y);
                dashTelem.addData("heading error", headingError);
                dashTelem.addData("heading translationalPower", headingPower);
                dashTelem.addData("translation error", translationError);
                dashTelem.addData("vel error", velError);
                dashTelem.addData("vel angle", velHeading);
                dashTelem.addData("angle to target", headingToTarget);
                dashTelem.addData("vel angle error", velHeadingError);
                dashTelem.addData("vel error sign to target", velErrorSignToTarget);
                dashTelem.addData("vel correction translationalPower", velCorrectionPower);
                dashTelem.addData("proportional translationalPower", proportionalPower);
                dashTelem.addData("translation translationalPower", translationalPower);
                dashTelem.addData("forward translationalPower", forwardPower);
                dashTelem.addData("strafe translationalPower", strafePower);
                dashTelem.addData("left front translationalPower", leftFrontPower);
                dashTelem.addData("right front translationalPower", rightFrontPower);
                dashTelem.update();

                StringBuilder telemetry = new StringBuilder();
                telemetry.append("Paused: ").append(pause).append('\n');
                telemetry.append("Target pose: ").append(targetPose).append('\n');
                telemetry.append("Target point index: ").append(targetPoint).append('\n');
                telemetry.append("Robot x: ").append(robotPose.position.x).append('\n');
                telemetry.append("Robot y: ").append(robotPose.position.y).append('\n');
                telemetry.append("Robot heading: ").append(robotPose.heading.toDouble()).append('\n');
                telemetry.append("Robot RotationToTarget: ").append(robotRotationToTarget.toDouble()).append('\n');
                telemetry.append("Pose error x: ").append(poseError.position.x).append('\n');
                telemetry.append("Pose error y: ").append(poseError.position.y).append('\n');
                telemetry.append("heading error: ").append(headingError).append('\n');
                telemetry.append("heading translationalPower: ").append(headingPower).append('\n');
                telemetry.append("translation error: ").append(translationError).append('\n');
                telemetry.append("vel error: ").append(velError).append('\n');
                telemetry.append("vel angle: ").append(velHeading).append('\n');
                telemetry.append("angle to target: ").append(headingToTarget).append('\n');
                telemetry.append("vel angle error: ").append(velHeadingError).append('\n');
                telemetry.append("vel error sign to target: ").append(velErrorSignToTarget).append('\n');
                telemetry.append("vel correction translationalPower: ").append(velCorrectionPower).append('\n');
                telemetry.append("proportional translationalPower: ").append(proportionalPower).append('\n');
                telemetry.append("translation translationalPower: ").append(translationalPower).append('\n');
                telemetry.append("forward translationalPower: ").append(forwardPower).append('\n');
                telemetry.append("strafe translationalPower: ").append(strafePower).append('\n');
                telemetry.append("left front translationalPower: ").append(leftFrontPower).append('\n');
                telemetry.append("right front translationalPower: ").append(rightFrontPower).append('\n');
                return telemetry.toString();
            }
        }

        return "";
    }
}
