package net.polarbub.ftc.hardware;

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ScannedDevices;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import net.polarbub.ftc.lib.SingletonHelper;
import net.polarbub.ftc.lib.TeamCodeWarningSource;

import java.util.Arrays;

import static net.polarbub.ftc.lib.OnCreateHandlers.*;

public class UsbScanner {
    public static UsbScanner getOrCreate(HardwareMap hardwareMap) {
        return SingletonHelper.getOrCreate(UsbScanner.class, () -> new UsbScanner(hardwareMap));
    }

//    @OpModeRegistrar
//    public static void register(OpModeManager manager) {
////        manager
//    }

    public final SerialNumber lynxUsbSerialNumber;
    public final SerialNumber webCamSerialNumber;

    private UsbScanner(HardwareMap hardwareMap) {
        try {
            SerialNumber localLynxUsbSerialNumber = null;
            SerialNumber localWebCamSerialNumber = null;

            if (Device.isRevControlHub()) {
                localLynxUsbSerialNumber = LynxConstants.SERIAL_NUMBER_EMBEDDED;
            }

            ScannedDevices devices = new HardwareDeviceManager(hardwareMap.appContext, getEventLoopManger().get()).scanForUsbDevices();
            for (SerialNumber sn : devices.keySet()) {
                DeviceManager.UsbDeviceType deviceType = devices.get(sn);
                if (deviceType == DeviceManager.UsbDeviceType.LYNX_USB_DEVICE) {
                    if (localLynxUsbSerialNumber != null) {
                        throw new RuntimeException("Multiple Expansion Hubs (Lynx USB devices) are connected at once. Please disconnect the extra one(s)");
                    }
                    localLynxUsbSerialNumber = sn;
                } else if (deviceType == DeviceManager.UsbDeviceType.WEBCAM) {
                    if (localWebCamSerialNumber != null) {
                        throw new RuntimeException("Multiple Cameras are connected at once. Please disconnect the extra one(s)");
                    }
                    localWebCamSerialNumber = sn;
                }
            }

            if (!Device.isRevControlHub()) {
                if (localLynxUsbSerialNumber == null) {
                    //TEST: Do these errors recheck when you rerun?
                    //TEST: Do these errors get flagged again when something has come unplugged after working once?
                    throw new RuntimeException("No Expansion Hub (Lynx USB device) found. Please check that there is a Expansion Hub connected");
                }
            }

            if (localWebCamSerialNumber == null) {
                RobotLog.ww("UsbScanner", "No webcam found!\n" + Arrays.toString(Thread.currentThread().getStackTrace()));
                TeamCodeWarningSource.getInstance().setWarning("No webcam found!");
            }

            lynxUsbSerialNumber = localLynxUsbSerialNumber;
            webCamSerialNumber = localWebCamSerialNumber;
        } catch (RobotCoreException e) {
            throw new RuntimeException(e);
        }
    }
}
