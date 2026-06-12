package org.ducktapeandaprayer.ftc.hardware;

public class AutoIntake {
    private final MainBotHardware hardware;

    public AutoIntake(MainBotHardware hardware) {
        this.hardware = hardware;
    }

    public void startIntaking() {
        MainBotHardware.Container container = this.hardware.getContainer();
        container.transferDrive.setPower(MainBotHardware.transferInPower);
        container.intakeDrive.setPower(MainBotHardware.intakeInPower);
        container.rampServo.setPosition(MainBotHardware.rampDownPos);
    }

    public boolean tick() {
        MainBotHardware.Container container = this.hardware.getContainer();
        //If transfer full stop it. If robot full stop intaking and rumble controller
        if(container.transferMiddleBB.getState() == BeamBreak.State.FULL && container.transferEndBB.getState() == BeamBreak.State.FULL) {
            container.transferDrive.setPower(0);

            if(container.intakeBB.getState() == BeamBreak.State.FULL) {
                container.intakeDrive.setPower(0);
                container.rampServo.setPosition(MainBotHardware.rampShootPos);
                return true;
            }
        }

        return false;

    }
}
