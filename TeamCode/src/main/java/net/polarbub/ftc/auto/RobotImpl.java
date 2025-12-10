package net.polarbub.ftc.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import net.polarbub.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

public class RobotImpl implements RobotInterface {
    public static int requiredAtSpeedSamples = 5;
    public static double launchTimeOut = 10;

    private final MainBotHardware hardware;
    private final Supplier<Boolean> isOpModeActive;



    public RobotImpl(MainBotHardware hardware, Supplier<Boolean> isOpModeActive) {
        this.hardware = hardware;
        this.isOpModeActive = isOpModeActive;
    }

    @Override
    public void launch(int numToLaunch, double speed) {
//        hardware.moverServo.setPosition(MainBotHardware.outServoPos);
//        hardware.moverServo2.setPosition(MainBotHardware.outServoPos);
//        hardware.launchDrive.setVelocity(-200);

        hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
        hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
        hardware.launchDrive.setVelocity(speed);

//        try {
//            Thread.sleep(500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }


        while (hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) > MainBotHardware.ballInKickerDist && hardware.launcherColorDistSensor2.getDistance(DistanceUnit.INCH) > MainBotHardware.ballInKickerDist2 && isOpModeActive.get()) {
            RobotLog.ii("RobotImpl", "0: " + (hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) > MainBotHardware.ballInKickerDist));
            RobotLog.ii("RobotImpl", "0: " + hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH));
            RobotLog.ii("RobotImpl", "1: " + (hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) > MainBotHardware.ballInKickerDist));
            RobotLog.ii("RobotImpl", "1: " + hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH));
            RobotLog.ii("RobotImpl", "");

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        hardware.moverDrive.setPower(0);
        while (!hardware.launchDrive.isAtTargetVelocity(21) && isOpModeActive.get()) {
            RobotLog.ii("RobotImpl", "Diff: " + Math.abs(hardware.launchDrive.getVelocity() - hardware.launchDrive.getTargetVelocity()));
            hardware.modules.clearBulkCache();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        RobotLog.ii("RobotImpl", "At speed start");

        //Wait a little longer to be safe
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        hardware.moverDrive.setPower(0);
        hardware.intakeDrive.setPower(0);
        hardware.kickerServo.setPosition(MainBotHardware.kickerServoUpPos);

        int speedDips = 0;
        int atSpeedSamples = requiredAtSpeedSamples + 1;
        int underSpeedSamples = requiredAtSpeedSamples + 1;

        //ADD: Timeout for launching
        ElapsedTime time = new ElapsedTime();
        time.reset();
        
        while (isOpModeActive.get() && speedDips < numToLaunch) {
            if(time.seconds() >= launchTimeOut) {
                RobotLog.ii("RobotImpl", "Timed out");
                break;
            }
            hardware.modules.clearBulkCache();
            if (hardware.launchDrive.isAtTargetVelocity()) {
                if(underSpeedSamples != requiredAtSpeedSamples) {
                    underSpeedSamples = 0;
                }

                if (atSpeedSamples == requiredAtSpeedSamples) {
                    atSpeedSamples++;

                    if(underSpeedSamples >= 2) {
                        //We were under speed
                        speedDips++;

                        RobotLog.ii("RobotImpl", "speed dip logged # " + speedDips);

                        if(speedDips >= numToLaunch) {
                            break;
                        }

                        hardware.kickerServo.setPosition(0);
                        hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
                        hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);

                        try {
                            Thread.sleep(400);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        while (hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) > MainBotHardware.ballInKickerDist && hardware.launcherColorDistSensor2.getDistance(DistanceUnit.INCH) > MainBotHardware.ballInKickerDist2 && isOpModeActive.get()) {
                            RobotLog.ii("RobotImpl", "0: " + (hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) > MainBotHardware.ballInKickerDist));
                            RobotLog.ii("RobotImpl", "0: " + hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH));
                            RobotLog.ii("RobotImpl", "1: " + (hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) > MainBotHardware.ballInKickerDist));
                            RobotLog.ii("RobotImpl", "1: " + hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH));
                            RobotLog.ii("RobotImpl", "");

                            try {
                                Thread.sleep(100);
                            } catch (InterruptedException e) {
                                throw new RuntimeException(e);
                            }
                        }

                        hardware.moverDrive.setPower(0);
                        hardware.intakeDrive.setPower(0);
                        hardware.kickerServo.setPosition(MainBotHardware.kickerServoUpPos);
                    }

                    underSpeedSamples = 0;
                } else if (atSpeedSamples < requiredAtSpeedSamples) {
                    atSpeedSamples++;
                    RobotLog.ii("RobotImpl", "At speed");
                }

                hardware.blinkin.setPattern(MainBotHardware.Patterns.correctSpeedNoVision);
            } else {
                atSpeedSamples = 0;

                if(underSpeedSamples < requiredAtSpeedSamples) {
                    underSpeedSamples++;
                    RobotLog.ii("RobotImpl", "Under speed inc to " + underSpeedSamples);
                } else {
                    RobotLog.ii("RobotImpl", "Under speed");
                }

                hardware.blinkin.setPattern(MainBotHardware.Patterns.wrongSpeedNoVisionPattern);
            }

            RobotLog.ii("RobotImpl", "Velocity: " + hardware.launchDrive.getVelocity());
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        hardware.launchDrive.setVelocity(0);
        hardware.moverDrive.setPower(0);
        hardware.intakeDrive.setPower(0);
        hardware.kickerServo.setPosition(0);

    }

    @Override
    public void intake() {
        hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
        hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
    }

    @Override
    public void stopIntake() {
        hardware.intakeDrive.setPower(0);
        hardware.moverDrive.setPower(0);
        hardware.launchDrive.setVelocity(0);
    }
}
