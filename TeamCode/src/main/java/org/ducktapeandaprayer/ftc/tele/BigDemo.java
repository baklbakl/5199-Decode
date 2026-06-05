package org.ducktapeandaprayer.ftc.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.TripleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.LynxModuleInitializer;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;

import static java.lang.Math.abs;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="DemoBot (Big)", group = MainBotConfig.demoGroupName)
public class BigDemo extends FastIterativeOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private List<VariableWatcher> variablesWatched = null;

    @Override
    protected void init_internal() {
        LynxModuleInitializer hardware = LynxModuleInitializer.getOrCreate(hardwareMap);

        leftFrontDrive = hardware.parentModule.createDcMotor(0);
        rightFrontDrive = hardware.parentModule.createDcMotor(1);
        leftRearDrive = hardware.parentModule.createDcMotor(2);
        rightRearDrive = hardware.parentModule.createDcMotor(3);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        variablesWatched = new ArrayList<>();
        try {
            variablesWatched.add(new TripleVariableWatcher<>(gamepad1, Gamepad.class.getDeclaredField("left_stick_x"), Gamepad.class.getDeclaredField("left_stick_y"), Gamepad.class.getDeclaredField("right_stick_x"), Float::equals,
                    (Float left_stick_x, Float left_stick_y, Float right_stick_x) -> {
                        double forward = -left_stick_y;
                        double strafe = left_stick_x;
                        double turning = right_stick_x;

                        double leftFrontPower  = forward + strafe + turning;
                        double rightFrontPower = forward - strafe - turning;
                        double leftBackPower   = forward - strafe + turning;
                        double rightBackPower  = forward + strafe - turning;

                        RobotLog.ii("BigDemo", "Powers: { lf: " + leftFrontPower + ", rf: " + rightFrontPower + ", lr: " + leftBackPower + ", rr: " + rightBackPower + "}");

                        // Normalize the values so no wheel power exceeds 100%
                        // This ensures that the robot maintains the desired motion.
                        double max = Math.max(Math.max(abs(leftBackPower), Math.max(abs(leftFrontPower), abs(rightFrontPower))), abs(rightBackPower));

                        if (max > 1.0) {
                            leftFrontPower  /= max;
                            rightFrontPower /= max;
                            leftBackPower   /= max;
                            rightBackPower  /= max;
                        }

                        leftFrontDrive.setPower(leftFrontPower);
                        leftRearDrive.setPower(leftBackPower);
                        rightFrontDrive.setPower(rightFrontPower);
                        rightRearDrive.setPower(rightBackPower);
                    }
            ));

        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    protected void init_loop_internal() {

    }

    @Override
    protected void start_internal() {

    }

    @Override
    protected void loop_internal() {
        for(VariableWatcher watcher : variablesWatched) {
            watcher.tick();
        }
        telemetry.update();
    }

    @Override
    protected void stop_internal() {

    }
}