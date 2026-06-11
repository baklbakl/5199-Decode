package org.ducktapeandaprayer.ftc.hardware;

public class AutoIntake {
    private final MainBotHardware hardware;

    public AutoIntake(MainBotHardware hardware) {
        this.hardware = hardware;
    }

    public void startIntaking() {
        hardware.transferDrive.setPower(MainBotHardware.transferInPower);
        hardware.intakeDrive.setPower(MainBotHardware.intakeInPower);
        hardware.rampServo.setPosition(MainBotHardware.rampDownPos);
    }

    public boolean tick() {
        //If transfer full stop it. If robot full stop intaking and rumble controller
        if(hardware.transferMiddleBB.getState() == BeamBreak.State.FULL && hardware.transferEndBB.getState() == BeamBreak.State.FULL) {
            hardware.transferDrive.setPower(0);

            if(hardware.intakeBB.getState() == BeamBreak.State.FULL) {
                hardware.intakeDrive.setPower(0);
                hardware.rampServo.setPosition(MainBotHardware.rampShootPos);
                return true;
            }
        }

        return false;

    }
}
