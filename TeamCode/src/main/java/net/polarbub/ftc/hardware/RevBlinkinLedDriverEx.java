package net.polarbub.ftc.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class RevBlinkinLedDriverEx extends RevBlinkinLedDriver {
    /**
     * RevBlinkinLedDriver
     *
     * @param controller A REV servo controller
     * @param port       the port that the driver is connected to
     */
    public RevBlinkinLedDriverEx(ServoControllerEx controller, int port) {
        super(controller, port);
    }



    @Override
    public void setPattern(BlinkinPattern pattern) {
        super.setPattern(pattern);
    }
}
