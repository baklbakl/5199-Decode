package org.ducktapeandaprayer.ftc.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

@TeleOp(name = "LauncherTuner", group = MainBotConfig.opmodeGroupName)
public class LauncherTuner extends FastIterativeOpMode {
    public static double internalControlkV = 0.0056;
    public static double internalControlkP = 0.05;
    public static int underSpeedRequired = 10;

    private MainBotHardware hardware = null;
    private List<VariableWatcher> variablesWatched = new ArrayList<>();
    private MultipleTelemetry mt = null;
    private PIDFCoefficients currentCoefficients = null;
    private Telemetry dashboardTelemetry = null;
    private ElapsedTime runtime = null;

    private enum ControlMode {
        MANUAL("Manual Control"),
        LYNX_PIDF("Lynx PIDF Control"),
        INTERNAL_PIDF("Internal PIDF Control");

        private final String stringValue;

        private static final ControlMode[] vals = values();

        public ControlMode next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }

        public String toString() {
            return stringValue;
        }

        ControlMode(String stringValue) {
            this.stringValue = stringValue;
        }
    }

    private ControlMode mode = ControlMode.LYNX_PIDF;
    private double targetVelocity = 0;

    @Override
    protected void init_internal() {
        //We don't enable bulk caching because it is faster to read a single value without it.
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        mt = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        currentCoefficients = new PIDFCoefficients(hardware.launchDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        runtime = new ElapsedTime();

        try {
            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("dpad_up"), Boolean::equals, (dpad_up) -> {
                if(dpad_up && mode != ControlMode.MANUAL) {
                    if(targetVelocity == MainBotHardware.teleOpLauncherNoVisionVelocity) {
                        targetVelocity = 0;
                    } else {
                        targetVelocity = MainBotHardware.teleOpLauncherNoVisionVelocity;

                    }

                    if(mode == ControlMode.LYNX_PIDF) {
                        hardware.launchDrive.setVelocity(targetVelocity);
                    }

                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("dpad_down"), Boolean::equals, (dpad_down) -> {
                if(dpad_down) {
                    hardware.launchDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MainBotHardware.launcherCoefficients);
                    currentCoefficients = new PIDFCoefficients(hardware.launchDrive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("cross"), Boolean::equals, (cross) -> {
                if(cross) {
                    mode = mode.next();
                    spinUpStartTime = -1;
                    speedDipStartTime = -1;

                    switch (mode) {
                        case MANUAL:
                            hardware.launchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            hardware.launchDrive.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                            break;
                        case LYNX_PIDF:
                            hardware.launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            targetVelocity = 0;
                            hardware.launchDrive.setVelocity(0);
                            break;
                        case INTERNAL_PIDF:
                            hardware.launchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            targetVelocity = 0;
                            break;
                    }
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("left_trigger"), gamepad1.getClass().getField("right_trigger"), Float::equals, (left_trigger, right_trigger) -> {
                if(mode == ControlMode.MANUAL) {
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

    private long spinUpStartTime = -1;
    private long speedDipStartTime = -1;
    //private boolean lastVelocityZero = false;
    private double lastVelocity = Double.NaN;
    private double lastSpinUpTime = 0;
    private double lastSpeedDipRecoveryTime = 0;
    private int underSpeedSamples = 0;

    @Override
    protected void loop_internal() {
        //Check for changes of watched variables and run the handler if they did
        for(VariableWatcher variableWatcher : variablesWatched) {
            variableWatcher.tick();
        }

        long thisNanos = runtime.nanoseconds();
        double velocity = hardware.launchDrive.getVelocity();

        mt.addLine(mode.toString());

        if(mode == ControlMode.MANUAL) {
            if(velocity != 0 && lastVelocity == 0) {
                spinUpStartTime = thisNanos;
            }
        } else {
            if(mode == ControlMode.INTERNAL_PIDF) {
                if(targetVelocity == 0) {
                    hardware.launchDrive.setPower(0);
                } else {
                    hardware.launchDrive.setPower(((targetVelocity - velocity) * internalControlkP + (targetVelocity) * internalControlkV) / hardware.voltageSensor.getVoltage());
                }
            } else {
                mt.addData("Current coefficients", currentCoefficients);
            }

            boolean atTargetSpeed = Math.abs(velocity) - Math.abs(targetVelocity) >= -1;

            if(targetVelocity != 0) {
                if(lastVelocity == 0 && spinUpStartTime == -1) {
                    spinUpStartTime = thisNanos;
                }

                if(atTargetSpeed) {
                    if(speedDipStartTime != -1 && underSpeedSamples >= underSpeedRequired) {
                        lastSpeedDipRecoveryTime = (thisNanos - speedDipStartTime) / 1000000000.0d;
                        speedDipStartTime = -1;
                    }

                    underSpeedSamples = 0;
                } else {
                    if(underSpeedSamples == 0) {
                        speedDipStartTime = thisNanos;
                    }
                    underSpeedSamples++;
                }
            }

            mt.addData("Target Vel", targetVelocity);
            mt.addData("Last speed dip recovery time", lastSpeedDipRecoveryTime);
        }

        if(spinUpStartTime != -1 && abs(velocity) >= abs(targetVelocity)) {
            lastSpinUpTime = (thisNanos - spinUpStartTime) / 1000000000.0d;
            spinUpStartTime = -1;
        }

        if(spinUpStartTime == -1) {
            mt.addData("Last spin up time", lastSpinUpTime);
        } else {
            mt.addLine("Spinning up ...");
        }

        mt.addData("Vel", velocity);
        lastVelocity = velocity;
        //These scale the graph window in the dashboard
        dashboardTelemetry.addData("Min", 0);
        dashboardTelemetry.addData("Max",  2000);

        mt.update();
    }

    @Override
    protected void stop_internal() {

    }
}
