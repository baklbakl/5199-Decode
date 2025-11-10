package net.polarbub.ftc.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

public class RobotImpl implements RobotInterface {
    public static int requiredAtSpeedSamples = 5;
    public static double safeBallDist = 1.2;

    private final MainBotHardware hardware;
    private final Supplier<Boolean> isOpModeActive;

    public RobotImpl(MainBotHardware hardware, Supplier<Boolean> isOpModeActive) {
        this.hardware = hardware;
        this.isOpModeActive = isOpModeActive;
    }

    @Override
    public void launch(int numToLaunch, double speed) {
        if(hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) < safeBallDist) {
            hardware.moverServo.setPosition(MainBotHardware.outServoPos);
            hardware.moverServo2.setPosition(MainBotHardware.outServoPos);
        }

        while(hardware.launcherColorDistSensor.getDistance(DistanceUnit.INCH) < safeBallDist) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        hardware.moverServo.setPosition(MainBotHardware.restingServoPos);
        hardware.moverServo2.setPosition(MainBotHardware.restingServoPos);

        hardware.launchDrive.setVelocity(speed);
        while(!hardware.launchDrive.isAtTargetVelocity(10)) {
            hardware.modules.clearBulkCache();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        //Wait a little longer to be safe
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        hardware.moverServo.setPosition(MainBotHardware.inServoPos);
        hardware.moverServo2.setPosition(MainBotHardware.inServoPos);
        hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);

        int speedDips = 0;
        int atSpeedSamples = requiredAtSpeedSamples + 1;

        //ADD: Timeout for launching
        while(isOpModeActive.get() && speedDips < numToLaunch) {
            hardware.modules.clearBulkCache();
            if(hardware.launchDrive.isAtTargetVelocity()) {
                if(atSpeedSamples == requiredAtSpeedSamples) {
                    atSpeedSamples++;
                    speedDips++;
                } else if(atSpeedSamples < requiredAtSpeedSamples) {
                    atSpeedSamples++;
                }

                hardware.blinkin.setPattern(MainBotHardware.correctSpeedNoVision);
            } else {
                atSpeedSamples = 0;
                hardware.blinkin.setPattern(MainBotHardware.wrongSpeedNoVision);
            }

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        hardware.launchDrive.setVelocity(0);
        hardware.intakeDrive.setPower(0);
        hardware.moverServo.setPosition(MainBotHardware.restingServoPos);
        hardware.moverServo2.setPosition(MainBotHardware.restingServoPos);

    }

    @Override
    public void intake() {
        hardware.moverServo.setPosition(MainBotHardware.inServoPos);
        hardware.moverServo2.setPosition(MainBotHardware.inServoPos);
        hardware.intakeDrive.setPower(MainBotHardware.inIntakePower);
    }

    @Override
    public void stopIntake() {
        hardware.intakeDrive.setPower(0);
        hardware.moverServo.setPosition(MainBotHardware.restingServoPos);
        hardware.moverServo2.setPosition(MainBotHardware.restingServoPos);
    }

    @Override
    public int getSpike() {
        return 0;
    }
}
