package net.polarbub.ftc.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import net.polarbub.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

public class RobotImpl implements RobotInterface {
    public static int requiredAtSpeedSamples = 10;
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
        hardware.launchDrive.setVelocity(-200);

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        while (hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) < MainBotHardware.ballInKickerDist) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

//        hardware.moverServo.setPosition(MainBotHardware.restingServoPos);
//        hardware.moverServo2.setPosition(MainBotHardware.restingServoPos);
        hardware.launchDrive.setVelocity(speed);
        while (!hardware.launchDrive.isAtTargetVelocity(10)) {
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

//        hardware.moverServo.setPosition(MainBotHardware.moverDrivePower);
//        hardware.moverServo2.setPosition(MainBotHardware.moverDrivePower);
        hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);

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

                    if(underSpeedSamples == requiredAtSpeedSamples) {
                        //We were under speed
                        speedDips++;
                    }

                    underSpeedSamples = 0;

                    RobotLog.ii("RobotImpl", "speed dip logged # " + speedDips);

                } else if (atSpeedSamples < requiredAtSpeedSamples) {
                    atSpeedSamples++;
                    RobotLog.ii("RobotImpl", "At speed");
                }

                hardware.blinkin.setPattern(MainBotHardware.correctSpeedNoVision);
            } else {
                atSpeedSamples = 0;

                if(underSpeedSamples < requiredAtSpeedSamples) {
                    underSpeedSamples++;
                    RobotLog.ii("RobotImpl", "Under speed inc to " + underSpeedSamples);
                } else {
                    RobotLog.ii("RobotImpl", "Under speed");
                }

                hardware.blinkin.setPattern(MainBotHardware.wrongSpeedNoVisionPattern);
            }


            RobotLog.ii("RobotImpl", "Velocity" + hardware.launchDrive.getVelocity());
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        hardware.launchDrive.setVelocity(0);
        hardware.intakeDrive.setPower(0);
//        hardware.moverServo.setPosition(MainBotHardware.restingServoPos);
//        hardware.moverServo2.setPosition(MainBotHardware.restingServoPos);

    }

    @Override
    public void intake() {
        hardware.moverDrive.setPower(MainBotHardware.moverDrivePower);
        hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
        hardware.launchDrive.setVelocity(-200);
    }

    @Override
    public void stopIntake() {
        hardware.intakeDrive.setPower(0);
        hardware.moverDrive.setPower(0);
        hardware.launchDrive.setVelocity(0);
    }
}
