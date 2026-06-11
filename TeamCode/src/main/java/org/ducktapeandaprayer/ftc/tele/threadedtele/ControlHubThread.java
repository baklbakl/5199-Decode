package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.TripleVariableWatcher;

import java.util.concurrent.atomic.AtomicReference;

import static java.lang.Math.abs;

public class ControlHubThread extends ThreadEx {
    //
    // Dash Options
    //
    public static Pose2d startPose = new Pose2d(70 - 7.5, - (70 - 9.65), Math.PI);

    //
    // Global Variables
    //
    private final MainBotHardware hardware;
    private final ThreadedTele threadedTele;

    public final AtomicReference<Pose2d> robotPose = new AtomicReference<>(null);

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

        hardware.leftFrontDrive.setPower(leftFrontPower);
        hardware.leftRearDrive.setPower(leftBackPower);
        hardware.rightFrontDrive.setPower(rightFrontPower);
        hardware.rightRearDrive.setPower(rightBackPower);
    }

    private void resetPose(boolean resetHeading) {
        Pose2d correctedStartPose = startPose;
        if(threadedTele.getAlliance() == Alliance.BLUE) {
            correctedStartPose = new Pose2d(new Vector2d(startPose.position.x, -startPose.position.y), startPose.heading);
        }
        float heading;
        if(resetHeading) {
            heading = (float) correctedStartPose.heading.toDouble();
        } else {
            heading = hardware.octoQuad.readLocalizerData().heading_rad;
        }
        hardware.setOctoQuadPose(correctedStartPose.position.x, correctedStartPose.position.y, heading);
    }

    public ControlHubThread(MainBotHardware hardwareArg, ThreadedTele threadedTele) {
        this.hardware = hardwareArg;
        this.threadedTele = threadedTele;
        hardware.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        try {
            //Movement
            super.variablesWatched.add(new TripleVariableWatcher<>(threadedTele.gamepad1, Gamepad.class.getDeclaredField("left_stick_x"), Gamepad.class.getDeclaredField("left_stick_y"), Gamepad.class.getDeclaredField("right_stick_x"), Float::equals,
                    (Float left_stick_x_f, Float left_stick_y_f, Float right_stick_x) -> {
                        computeSetWheelSpeeds(-left_stick_y_f, left_stick_x_f, right_stick_x);
                    }
            ));

            super.variablesWatched.add(new SingleVariableWatcher<>(threadedTele.gamepad1, Gamepad.class.getField("options"), Boolean::equals, (Boolean options) -> {
                if(options) {
                    resetPose(false);
                }
            }));

            super.variablesWatched.add(new SingleVariableWatcher<>(threadedTele.gamepad1, Gamepad.class.getField("share"), Boolean::equals, (Boolean share) -> {
                if(share) {
                    resetPose(true);
                }
            }));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void run() {
        resetPose(true);
        super.run();
    }

    public void tick() {
        OctoQuad.LocalizerDataBlock block = hardware.readOctoQuadLocalizerData();
        Pose2d newPose = MainBotHardware.makeOctoQuadPose(block);

        telemetry.append("Octoquad x: ");
        telemetry.append(newPose.position.x);
        telemetry.append("\nOctoquad y: ");
        telemetry.append(newPose.position.y);
        telemetry.append("\nOctoquad heading: ");
        telemetry.append(newPose.heading.toDouble());
        telemetry.append("\nOctoquad heading vel: ");
        telemetry.append(block.velHeading_radS);
        telemetry.append("\n");

        robotPose.set(newPose);

        updateTelemetry();
        hardware.modules.parentModule.module.clearBulkCache();
    }
}
