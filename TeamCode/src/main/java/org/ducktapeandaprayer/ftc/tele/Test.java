package org.ducktapeandaprayer.ftc.tele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

@TeleOp(name = "Test", group = MainBotConfig.opmodeGroupName)
public class Test extends FastIterativeOpMode {
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

    private final static List<Pose2d> waypoints = List.of(
            new Pose2d(25, 0, 0),
            new Pose2d(25, 25, 0),
            new Pose2d(0, 0, 0),
            new Pose2d(50, 0, 0),
            new Pose2d(50, 50, 0),
            new Pose2d(0, 0, 0)
    );

    private final static Pose2d startPoint = new Pose2d(0, 0, 0);

    private ElapsedTime runtime = new ElapsedTime();
    private ArrayList<VariableWatcher> variableWatchers = new ArrayList<>();
    private MainBotHardware hardware;
    private MainBotHardware.Container container;
    private boolean run = false;
    private int targetPoint = 0;

    public void init_internal() {
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        container = hardware.getContainer();
        hardware.setOctoQuadPose(startPoint);
        container.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        container.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        container.leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        container.rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            variableWatchers.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("square"), Gamepad.class.getField("circle"), Boolean::equals, (Boolean square, Boolean circle) -> {
                if(square == circle) {
                    return;
                }

                if(square) {
                    run = !run;
                    if(!run) {
                        container.leftFrontDrive.setPower(0);
                        container.rightFrontDrive.setPower(0);
                        container.leftRearDrive.setPower(0);
                        container.rightRearDrive.setPower(0);
                    }
                } else if(circle && !run) {
                    hardware.setOctoQuadPose(startPoint);
                    targetPoint = 0;
                }
            }));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop_internal() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start_internal() {
        runtime.reset();
    }

    private void computeSetWheelSpeeds(double forward, double strafe, double turning) {

        double leftFrontPower  = forward + strafe + turning;
        double rightFrontPower = forward - strafe - turning;
        double leftBackPower   = forward - strafe + turning;
        double rightBackPower  = forward + strafe - turning;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.max(abs(leftBackPower), Math.max(abs(leftFrontPower), abs(rightFrontPower))), abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        container.leftFrontDrive.setPower(leftFrontPower);
        container.leftRearDrive.setPower(leftBackPower);
        container.rightFrontDrive.setPower(rightFrontPower);
        container.rightRearDrive.setPower(rightBackPower);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop_internal() {
        container.modules.clearBulkCache();
        for(VariableWatcher watcher : variableWatchers) {
            watcher.tick();
        }

        double voltage = container.voltageSensor.getVoltage();
        OctoQuad.LocalizerDataBlock dataBlock = hardware.readOctoQuadLocalizerData();
        Pose2d robotPose = MainBotHardware.makeOctoQuadPose(dataBlock);
        PoseVelocity2d robotVelocity = new PoseVelocity2d(new Vector2d(dataBlock.velX_mmS / 25.4, dataBlock.velX_mmS / 25.4), dataBlock.velHeading_radS);
        Pose2d poseError = waypoints.get(targetPoint).minusExp(robotPose);
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
            if(targetPoint >= waypoints.size()) {
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
        //
        //double kSVoltageComp = kS / voltage;
        //
        //leftFrontPower += Math.signum(leftFrontPower) * kSVoltageComp;
        //rightFrontPower += Math.signum(rightFrontPower) * kSVoltageComp;
        //leftBackPower += Math.signum(leftBackPower) * kSVoltageComp;
        //rightBackPower += Math.signum(rightBackPower) * kSVoltageComp;

        if(run) {
            container.leftFrontDrive.setPower(leftFrontPower);
            container.rightFrontDrive.setPower(rightFrontPower);
            container.leftRearDrive.setPower(leftBackPower);
            container.rightRearDrive.setPower(rightBackPower);
        }

        telemetry.addData("run", run);
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

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop_internal() {
    }
}
