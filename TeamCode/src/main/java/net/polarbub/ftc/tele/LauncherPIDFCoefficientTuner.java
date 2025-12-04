package net.polarbub.ftc.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import net.polarbub.ftc.lib.variablewatcher.DoubleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.SingleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.VariableWatcher;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

@TeleOp(name = "LauncherPIDFCoefficientTuner", group = MainBotConfig.opmodeGroupName)
public class LauncherPIDFCoefficientTuner extends FastIterativeOpMode {
    private MainBotHardware hardware = null;
    private List<VariableWatcher> variablesWatched = new ArrayList<>();
    private MultipleTelemetry mt = null;
    private PIDFCoefficients currentCoefficients = null;
    private Telemetry dashboardTelemetry = null;
    private ElapsedTime runtime = null;

    @Override
    protected void init_internal() {
        //We don't enable bulk caching because it is faster to read a single value without it.
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        mt = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        currentCoefficients = hardware.launchDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        runtime = new ElapsedTime();

        try {
            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("dpad_up"), Boolean::equals, (dpad_up) -> {
                if(dpad_up) {
                    if(hardware.launchDrive.getTargetVelocity() == MainBotHardware.teleOpLauncherNoVisionVelocity) {
                        hardware.launchDrive.setVelocity(0);
                    } else {
                        hardware.launchDrive.setVelocity(MainBotHardware.teleOpLauncherNoVisionVelocity);
                    }
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("dpad_down"), Boolean::equals, (dpad_down) -> {
                if(dpad_down) {
                    hardware.launchDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MainBotHardware.launcherCoefficients);
                    currentCoefficients = MainBotHardware.launcherCoefficients;
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("cross"), Boolean::equals, (cross) -> {
                if(cross) {
                    DcMotor.RunMode mode = hardware.launchDrive.getMode();
                    if(mode == DcMotor.RunMode.RUN_USING_ENCODER) {
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hardware.launchDrive.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                    } else if(mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        hardware.launchDrive.setVelocity(0);
                    }
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("left_trigger"), gamepad1.getClass().getField("right_trigger"), Float::equals, (left_trigger, right_trigger) -> {
                if(hardware.launchDrive.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    hardware.launchDrive.setPower(left_trigger - right_trigger);
                }
            }));

        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    protected void init_loop_internal() {

    }

    @Override
    protected void start_internal() {
        runtime.reset();
    }

    private long startTime = -1;
    private boolean lastVelocityZero = false;
    private double lastSpinUpTime = 0;

    @Override
    protected void loop_internal() {
        //Check for changes of watched variables and run the handler if they did
        for(VariableWatcher variableWatcher : variablesWatched) {
            variableWatcher.tick();
        }

        long thisNanos = runtime.nanoseconds();
        double velocity = hardware.launchDrive.getVelocity();
        double targetVelocity;

        DcMotor.RunMode mode = hardware.launchDrive.getMode();

        if(mode == DcMotor.RunMode.RUN_USING_ENCODER) {
            targetVelocity = hardware.launchDrive.getTargetVelocity();
            if(targetVelocity != 0 && lastVelocityZero && startTime == -1) {
                startTime = thisNanos;
            }
            lastVelocityZero = targetVelocity == 0;
            mt.addData("Current coefficients", currentCoefficients);
            mt.addData("Target Vel", targetVelocity);
        } else {
            targetVelocity = MainBotHardware.teleOpLauncherNoVisionVelocity;

            if(velocity != 0 && lastVelocityZero) {
                startTime = thisNanos;
            }
            lastVelocityZero = velocity == 0;
            mt.addLine("Manual Control");
        }

        if(startTime != -1 && abs(velocity) >= abs(targetVelocity)) {
            lastSpinUpTime = (thisNanos - startTime) / 1000000000.0d;
            startTime = -1;
        }

        if(startTime == -1) {
            mt.addData("Last spin up time", lastSpinUpTime);
        } else {
            mt.addLine("Spinning up ...");
        }

        mt.addData("Vel", hardware.launchDrive.getVelocity());
        //These scale the graph window in the dashboard
        dashboardTelemetry.addData("Min", 0);
        dashboardTelemetry.addData("Max", 2000);

        mt.update();
    }

    @Override
    protected void stop_internal() {

    }
}
