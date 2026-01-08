package net.polarbub.ftc.tele.threadedtele;

import com.qualcomm.robotcore.hardware.Gamepad;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.lib.ThreadEx;
import net.polarbub.ftc.lib.variablewatcher.TripleVariableWatcher;

import static java.lang.Math.abs;

public class ControlHubThread extends ThreadEx {
    private final MainBotHardware hardware;

    public ControlHubThread(MainBotHardware hardwareArg, Gamepad gamepad1) {
        this.hardware = hardwareArg;

        try {
            //Movement
            super.variablesWatched.add(new TripleVariableWatcher<>(gamepad1, Gamepad.class.getDeclaredField("left_stick_x"), Gamepad.class.getDeclaredField("left_stick_y"), Gamepad.class.getDeclaredField("right_stick_x"), Float::equals,
                    (Float left_stick_x_f, Float left_stick_y_f, Float turning_f) -> {

                        // Note: pushing stick forward gives negative value
                        double leftFrontPower  = -left_stick_y_f + left_stick_x_f + turning_f;
                        double rightFrontPower = -left_stick_y_f - left_stick_x_f - turning_f;
                        double leftBackPower   = -left_stick_y_f - left_stick_x_f + turning_f;
                        double rightBackPower  = -left_stick_y_f + left_stick_x_f - turning_f;

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
            ));


        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    public void tick() {

    }
}
