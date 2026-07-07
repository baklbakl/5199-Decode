package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Follower {
    public static double kPTranslation = 0.9;
    public static double kPHeading = 5;
    public static double headingDoneThresh = 0.04;
    public static double headingNoPowerThresh = 0.02;
    public static double kS = 1;
    public static double doneThresh = 1;
    public static double waypointDoneThresh = 2.3;
    public static double kDfake = 0.165;
    public static double velocityPower = 1;
    public static double strafeMultiplier = 1.4;
    public static double brakeDistance = 45;

    private final MainBotHardware hardware;
    private List<Pose2d> points = null;
    private boolean pause = false;
    private int targetPoint = 0;
    private boolean holdEnd = false;
    private boolean enableTelemetry = false;
    private double restOfPathDistance = Double.NaN;

    public Follower(MainBotHardware hardware) {
        this.hardware = hardware;
    }

    public void followTrajectory(List<Pose2d> points, boolean holdEnd) {
        this.points = new ArrayList<>();
        //We put the data in our own arraylist so it won't get changed under our feet
        this.points.addAll(points);
        this.holdEnd = holdEnd;
        targetPoint = 0;

        restOfPathDistance = 0;
        for (int i = 1; i < points.size(); i++) {
            restOfPathDistance += points.get(i - 1).position.minus(points.get(i).position).norm();
        }
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
        if(points != null && dataBlock != null) {
            MainBotHardware.Container container = hardware.getContainer();
            double voltage = container.voltageSensor.getVoltage();
            Pose2d targetPose = points.get(targetPoint);
            Pose2d robotPose = MainBotHardware.makeOctoQuadPose(dataBlock);
            Pose2d poseError = new Pose2d(targetPose.position.x - robotPose.position.x, targetPose.position.y - robotPose.position.y, targetPose.heading.minus(robotPose.heading));

            //Simple proportional controller for translational
            double translationError = poseError.position.norm();
            double translationalPower = /*kS + */translationError * kPTranslation;

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
            double brakeDistanceLeft = brakeDistance - (translationError + restOfPathDistance);
            if(brakeDistanceLeft > 0) {
                velCorrectionPower = -velErrorSignToTarget * Math.pow(velError * kDfake, velocityPower) * (brakeDistanceLeft / brakeDistance);
                translationalPower += velCorrectionPower;
            } else {
                velCorrectionPower = 0;
            }

            //Simple kP for heading
            double headingError = poseError.heading.toDouble();
            double headingPower = -1 * headingError * kPHeading;

            if(Math.abs(headingError) < headingNoPowerThresh) {
                headingPower = 0;
            }

            //If our translation has finished then move onto the next point, hold the end point, or finish the path.
            // Right now we don't wait for heading to achieve its setpoint before moving on.
            boolean onLastPath = restOfPathDistance == 0;
            if(Math.abs(translationError) < (onLastPath ? doneThresh : waypointDoneThresh) && (!onLastPath || Math.abs(headingError) < headingDoneThresh)) {
                targetPoint++;
                if(targetPoint >= points.size()) {
                    targetPoint--;
                    if(!holdEnd) {
                        points = null;
                        headingPower = 0;
                        restOfPathDistance = Double.NaN;
                    }
                    translationalPower = 0;
                } else {
                    restOfPathDistance -= points.get(targetPoint - 1).position.minus(points.get(targetPoint).position).norm();
                    if(targetPoint + 1 == points.size()) {
                        if(Math.abs(restOfPathDistance) > 0.01) {
                            throw new RuntimeException("Math no work. restOfPathDistance should be zero for last path segment");
                        }
                        restOfPathDistance = 0;
                    }
                }
            }

            //Apply kS to rotation if there is no kS from translation (translation power is zero)
            //if(translationalPower == 0) {
            //    headingPower += Math.signum(headingPower) * kS;
            //}

            double voltageCompTranslationPower = translationalPower / voltage;
            //Rotate the direction of translation required to the robot's frame. Rotation2d.times() adds two rotations.
            // Rotation2d.inverse() is a unary minus, so in total we get headingToTarget - robotPose.heading;
            Rotation2d robotRotationToTarget = Rotation2d.fromDouble(headingToTarget).times(robotPose.heading.inverse());

            //Find the forward and strafe components of the translation. .real and .imag are just cos() and sin().
            // Roadrunner's rotation class stores rotations as the sin() and cos() of their angle
            double forwardPower = robotRotationToTarget.real * voltageCompTranslationPower;
            double strafePower = robotRotationToTarget.imag * voltageCompTranslationPower * strafeMultiplier;
            double voltageCompHeadingPower = headingPower / voltage;
            double voltageCompKS = kS / voltage;

            //Cap our wheel power at one while making sure all the wheels retain their relative power levels
            double maxPower = Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(voltageCompHeadingPower);
            double maxAllowedPower = (1 - voltageCompKS);
            if(maxPower > maxAllowedPower) {
                forwardPower *= maxAllowedPower / maxPower;
                strafePower *= maxAllowedPower / maxPower;
                headingPower *= maxAllowedPower / maxPower;
            }

            //Standard Mecanum kinematics
            double leftFrontPower  = forwardPower - strafePower + voltageCompHeadingPower;
            double rightFrontPower = forwardPower + strafePower - voltageCompHeadingPower;
            double leftBackPower   = forwardPower + strafePower + voltageCompHeadingPower;
            double rightBackPower  = forwardPower - strafePower - voltageCompHeadingPower;

            leftFrontPower += voltageCompKS * Math.signum(leftFrontPower);
            rightFrontPower += voltageCompKS * Math.signum(rightFrontPower);
            leftBackPower += voltageCompKS * Math.signum(leftBackPower);
            rightBackPower += voltageCompKS * Math.signum(rightBackPower);

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
                dashTelem.addData("heading power", headingPower);
                dashTelem.addData("rest of path distance", restOfPathDistance);
                dashTelem.addData("translation error", translationError);
                dashTelem.addData("vel error", velError);
                dashTelem.addData("vel angle", velHeading);
                dashTelem.addData("angle to target", headingToTarget);
                dashTelem.addData("vel angle error", velHeadingError);
                dashTelem.addData("vel error sign to target", velErrorSignToTarget);
                dashTelem.addData("vel correction translationalPower", velCorrectionPower);
                dashTelem.addData("translation power", translationalPower);
                dashTelem.addData("forward translationalPower", forwardPower);
                dashTelem.addData("strafe translationalPower", strafePower);
                dashTelem.addData("left front power", leftFrontPower);
                dashTelem.addData("right front power", rightFrontPower);
                dashTelem.update();

                String telemetry = "Paused: " + pause + '\n' +
                        "Target pose: " + targetPose + '\n' +
                        "Target point index: " + targetPoint + '\n' +
                        "Robot x: " + robotPose.position.x + '\n' +
                        "Robot y: " + robotPose.position.y + '\n' +
                        "Robot heading: " + robotPose.heading.toDouble() + '\n' +
                        "Robot RotationToTarget: " + robotRotationToTarget.toDouble() + '\n' +
                        "Pose error x: " + poseError.position.x + '\n' +
                        "Pose error y: " + poseError.position.y + '\n' +
                        "heading error: " + headingError + '\n' +
                        "heading translationalPower: " + headingPower + '\n' +
                        "rest of path distance: " + restOfPathDistance + "\n" +
                        "translation error: " + translationError + '\n' +
                        "vel error: " + velError + '\n' +
                        "vel angle: " + velHeading + '\n' +
                        "angle to target: " + headingToTarget + '\n' +
                        "vel angle error: " + velHeadingError + '\n' +
                        "vel error sign to target: " + velErrorSignToTarget + '\n' +
                        "vel correction translationalPower: " + velCorrectionPower + '\n' +
                        "translation translationalPower: " + translationalPower + '\n' +
                        "forward translationalPower: " + forwardPower + '\n' +
                        "strafe translationalPower: " + strafePower + '\n' +
                        "left front translationalPower: " + leftFrontPower + '\n' +
                        "right front translationalPower: " + rightFrontPower + '\n';
                return telemetry;
            }
        }

        return "";
    }
}
