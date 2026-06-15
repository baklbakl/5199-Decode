package org.ducktapeandaprayer.ftc.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class AutoShoot {
    public static double shootingTimeout = 2.0;
    public static double lastBallTime = 0.15;
    public static double shootTransferOnDelay = 0.3;
    public static double singleShootHighRampDelay = 0.01;


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
        double time = timer.time();
        shootingStartTimeForTransfer = time;
        shootingStartTimeForTimeout = time;
        shootingStartTimeForMeasurement = time;
        shootingStartTimeForRamp = Double.NaN;
        rampLastBallPosStartTime = Double.NaN;

        MainBotHardware.Container container = this.hardware.getContainer();
        if(Math.abs(container.rampServo.getPosition() - MainBotHardware.rampShootPos) < 0.01) {
            container.transferDrive.setPower(MainBotHardware.transferInPower);
            container.intakeDrive.setPower(MainBotHardware.intakeInPower);
            shootingStartTimeForTransfer = Double.NaN;
            shootingStartTimeForRamp = time;
        } else {
            container.rampServo.setPosition(MainBotHardware.rampShootPos);
            container.transferDrive.setPower(0);
            container.intakeDrive.setPower(0);
        }
    }

    public void stopShooting() {
        MainBotHardware.Container container = this.hardware.getContainer();
        container.transferDrive.setPower(0);
        container.intakeDrive.setPower(0);
        container.rampServo.setPosition(MainBotHardware.rampDownPos);
    }

    //Returns true when done
    public boolean tick() {
        MainBotHardware.Container container = this.hardware.getContainer();
        double time = timer.seconds();
        //Shooting start time is Double.NaN when not waiting to start the transfer and 1 - NaN = NaN and NaN >= any number = false, so this works
        if(time - shootingStartTimeForTransfer >= shootTransferOnDelay) {
            container.transferDrive.setPower(MainBotHardware.transferInPower);
            container.intakeDrive.setPower(MainBotHardware.intakeInPower);
            shootingStartTimeForTransfer = Double.NaN;
            shootingStartTimeForRamp = time;
        }

        if(time - shootingStartTimeForRamp >= singleShootHighRampDelay) {
            if(container.intakeBB.getState() == BeamBreak.State.EMPTY && container.transferMiddleBB.getState() == BeamBreak.State.EMPTY) {
                container.rampServo.setPosition(MainBotHardware.rampLastBallPos);
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

        if(Math.abs(container.rampServo.getPosition() - MainBotHardware.rampLastBallPos) < 0.01
                && time - rampLastBallPosStartTime > lastBallTime) {
            rampLastBallPosStartTime = Double.NaN;
            shootingStartTimeForTimeout = Double.NaN;
            RobotLog.ii("AutoShoot", "Shooting finished in " + (time - shootingStartTimeForMeasurement));
            return true;
        }

        return false;
    }
}
