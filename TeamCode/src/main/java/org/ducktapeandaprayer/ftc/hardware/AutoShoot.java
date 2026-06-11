package org.ducktapeandaprayer.ftc.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class AutoShoot {
    public static double shootingTimeout = 2.0;
    public static double lastBallTime = 0.6;

    public final MainBotHardware hardware;
    private final ElapsedTime timer = new ElapsedTime();
    private double shootingStartTimeForTransfer = Double.NaN;
    private double shootingStartTimeForRamp = Double.NaN;
    private double shootingStartTimeForTimeout = Double.NaN;
    private double shootingStartTimeForMeasurement = Double.NaN;
    private double rampLastBallPosStartTime = Double.NaN;

    public AutoShoot(MainBotHardware hardware) {
        this.hardware = hardware;
    }

    public void startShooting() {
        shootingStartTimeForTransfer = timer.time();
        shootingStartTimeForRamp = shootingStartTimeForTransfer;
        shootingStartTimeForTimeout = shootingStartTimeForTransfer;
        shootingStartTimeForMeasurement = shootingStartTimeForTransfer;
        rampLastBallPosStartTime = Double.NaN;
        if(Math.abs(hardware.rampServo.getPosition() - MainBotHardware.rampShootPos) < 0.01) {
            hardware.transferDrive.setPower(MainBotHardware.transferInPower);
            hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
            shootingStartTimeForTransfer = Double.NaN;
        } else {
            hardware.rampServo.setPosition(MainBotHardware.rampShootPos);
            hardware.transferDrive.setPower(0);
            hardware.intakeDrive.setPower(0);
        }
    }

    public void stopShooting() {
        hardware.transferDrive.setPower(0);
        hardware.intakeDrive.setPower(0);
        hardware.rampServo.setPosition(MainBotHardware.rampDownPos);
    }

    //Returns true when done
    public boolean tick() {
        double time = timer.seconds();
        //Shooting start time is Double.NaN when not waiting to start the transfer and 1 - NaN = NaN and NaN >= any number = false, so this works
        if(time - shootingStartTimeForTransfer >= MainBotHardware.shootTransferOnDelay) {
            hardware.transferDrive.setPower(MainBotHardware.transferInPower);
            hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
            shootingStartTimeForTransfer = Double.NaN;
        }

        if(time - shootingStartTimeForRamp >= MainBotHardware.singleShootHighRampDelay) {
            if(hardware.intakeBB.getState() == BeamBreak.State.EMPTY && hardware.transferMiddleBB.getState() == BeamBreak.State.EMPTY) {
                hardware.rampServo.setPosition(MainBotHardware.rampLastBallPos);
                shootingStartTimeForRamp = Double.NaN;
                rampLastBallPosStartTime = time;
            } else {
                shootingStartTimeForRamp = time;
                rampLastBallPosStartTime = Double.NaN;
            }
        }

        if(time - shootingStartTimeForTimeout > shootingTimeout) {
            RobotLog.ii("AutoShoot", "Shooting time out!");
            shootingStartTimeForTimeout = Double.NaN;
            return true;
        }

        if(Math.abs(hardware.rampServo.getPosition() - MainBotHardware.rampLastBallPos) < 0.01
                && time - rampLastBallPosStartTime > lastBallTime) {
            rampLastBallPosStartTime = Double.NaN;
            shootingStartTimeForTimeout = Double.NaN;
            RobotLog.ii("AutoShoot", "Shooting finished in " + (time - shootingStartTimeForMeasurement));
            return true;
        }

        return false;
    }
}
