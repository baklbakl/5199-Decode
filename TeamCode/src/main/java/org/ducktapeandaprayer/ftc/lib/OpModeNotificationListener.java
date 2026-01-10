package org.ducktapeandaprayer.ftc.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.hardware.UsbScanner;

public class OpModeNotificationListener implements OpModeManagerNotifier.Notifications {
    public static final OpModeNotificationListener instance = new OpModeNotificationListener();
    private OpModeNotificationListener() {}

    @Override
    public void onOpModePreInit(OpMode opMode) {
        //Force a rescan of the usb devices on opmode init
        if(!(opMode instanceof  OpModeManagerImpl.DefaultOpMode)) {
            RobotLog.ii("OpModeNotificationListener", "Rescanning for opmode: " + opMode);
            UsbScanner.rescan(opMode.hardwareMap);
        }
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        RobotLog.ii("ThreadedTele", "onOpModePostStop called");
    }
}
