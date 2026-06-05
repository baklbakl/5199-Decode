package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.apache.commons.lang3.tuple.Pair;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.SingleVariableWatcher;

public class ExpansionHubThread extends ThreadEx {
    //
    // Dash options
    //

    public static long launcherVelocityCheckInterval = 200;

    //
    // Globals
    //

    public Boolean autoSpeedDebInfo = false;

    private final MainBotHardware hardware;
    private final ThreadedTele threadedTele;

    private String autoSpeedTelemetry = "";

    private enum LauncherControlMethod {
        MANUAL,
        PRESET
    }

    private LauncherControlMethod launcherControlMethod;

    private static double launchSpeedFromDist(double dist) {
        synchronized (MainBotHardware.launchSpeedMapping) {
            Pair<Double, Double> lowestPair = MainBotHardware.launchSpeedMapping.get(0);
            if(dist < lowestPair.getKey()) {
                return Double.NaN;
            }


            for(int i = 0; i < MainBotHardware.launchSpeedMapping.size(); i++) {
                Pair<Double, Double> p = MainBotHardware.launchSpeedMapping.get(i);
                if(dist == p.getKey()) {
                    return p.getValue();
                }

                if(dist < p.getKey()) {
                    //Previous one must have been greater, so we know we are between the two closest points,
                    // so we interpolate with a weighted average
                    Pair<Double, Double> last = MainBotHardware.launchSpeedMapping.get(i-1);
                    double deltaDist = p.getKey() - last.getKey();
                    double offset = dist - last.getKey();
                    double weightLast = (deltaDist - offset) / deltaDist;
                    double weightThis = offset / deltaDist;


                    return weightLast * last.getValue() + weightThis * p.getValue();
                }
            }
        }

        return Double.NaN;
    }

    private void startLauncher() {
        if(launcherControlMethod == LauncherControlMethod.PRESET) {
            hardware.launchDrive.setVelocity(MainBotHardware.teleOpLauncherNoVisionVelocity);
        } else {
            throw new RuntimeException("Oops unaccounted for launcher control method \"" + launcherControlMethod + "\"");
        }
    }

    private void stopLauncher() {
        hardware.launchDrive.setVelocity(0);
    }

    public ExpansionHubThread(MainBotHardware hardwareArg, ThreadedTele threadedTele, ControlHubThread controlHubThread, Gamepad gamepad1, OptionDriver optionDriver) {
        this.hardware = hardwareArg;
        this.threadedTele = threadedTele;

        hardware.modules.childModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        try {
            //optionDriver.add(new BooleanOption("Auto Speed debug info", "dpad_right", gamepad1, this.getClass().getField("autoSpeedDebInfo"), this));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_bumper"), Gamepad.class.getField("right_bumper"), Boolean::equals, (Boolean left_bumper, Boolean right_bumper) -> {
                if(left_bumper == right_bumper) {
                    hardware.moverDrive.setPower(0);
                    hardware.intakeDrive.setPower(0);
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoDownPos);
                } else if(left_bumper) {
                    hardware.moverDrive.setPower(MainBotHardware.moverInPower);
                    hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoDownPos);
                } else if(right_bumper) {
                    hardware.moverDrive.setPower(MainBotHardware.moverShootPower);
                    hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoShootPos);
                } else {
                    throw new RuntimeException("Oopsy. Something bad happened. Never should be here");
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("options"), (Boolean options) -> {
                if(options) {
                    hardware.kickerServo.setPosition(MainBotHardware.kickerLastBallPose);
                } else if(gamepad1.right_bumper) {
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoShootPos);
                } else {
                    hardware.kickerServo.setPosition(MainBotHardware.kickerServoDownPos);
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, Gamepad.class.getField("triangle"), (Boolean triangle) -> {
                if(triangle && launcherControlMethod != LauncherControlMethod.MANUAL) {
                    if(hardware.launchDrive.getTargetVelocity() == 0) {
                        startLauncher();
                    } else {
                        stopLauncher();
                    }
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_trigger"), Gamepad.class.getField("right_trigger"), Float::equals, (Float left_trigger, Float right_trigger) -> {
                if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                    float power = left_trigger - right_trigger;
                    hardware.launchDrive.setPower(power);
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("left_stick_button"), Gamepad.class.getField("right_stick_button"), Boolean::equals, (Boolean left_stick_button, Boolean right_stick_button) -> {
                float power = gamepad1.left_trigger - gamepad1.right_trigger;
                if(left_stick_button && right_stick_button) {
                    if(launcherControlMethod == LauncherControlMethod.MANUAL) {
                        launcherControlMethod = LauncherControlMethod.PRESET;
                        hardware.launchDrive.setVelocity(0);
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    } else {
                        launcherControlMethod = LauncherControlMethod.MANUAL;
                        hardware.launchDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        hardware.launchDrive.setPower(power);
                    }
                }
            }));

        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void run() {
        launcherControlMethod = LauncherControlMethod.PRESET;
        super.run();
    }

    public void tick() {
        //if(hardware.moverDrive.getPower() > 0) {
        //    if(hardware.childLauncherSwitch.getState()) {
        //        hardware.moverDrive.setPower(MainBotHardware.moverInPower);
        //    } else {
        //        hardware.moverDrive.setPower(MainBotHardware.moverShootPower);
        //    }
        //}

        hardware.modules.childModule.module.clearBulkCache();

        telemetry.append("Launcher Target Speed: ");
        telemetry.append(hardware.launchDrive.getTargetVelocity());
        telemetry.append("\nLauncher Current Speed: ");
        telemetry.append(hardware.launchDrive.getVelocity());
        telemetry.append("\n\n");

        if (autoSpeedDebInfo) {
            telemetry.append(autoSpeedTelemetry);
            telemetry.append("\n");
        }
        updateTelemetry();
    }
}
