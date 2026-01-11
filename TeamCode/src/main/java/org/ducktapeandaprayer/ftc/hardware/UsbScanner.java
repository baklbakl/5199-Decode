package org.ducktapeandaprayer.ftc.hardware;

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ScannedDevices;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import org.ducktapeandaprayer.ftc.lib.SingletonHelper;
import org.ducktapeandaprayer.ftc.lib.TeamCodeWarningSource;

import java.util.Arrays;

import static org.ducktapeandaprayer.ftc.lib.OnCreateHandlers.*;

public class UsbScanner {
    public static UsbScanner getOrCreate(HardwareMap hardwareMap) {
        return SingletonHelper.getOrCreate(UsbScanner.class, () -> new UsbScanner(hardwareMap));
    }

    public static void clearInstance() {
        SingletonHelper.clear(UsbScanner.class);
    }

    public final SerialNumber lynxParentUsbSerialNumber;
    public final SerialNumber lynxChildUsbSerialNumber;
    public final SerialNumber webCamSerialNumber;

    private UsbScanner(HardwareMap hardwareMap) {
        try {
            SerialNumber localLynxParentUsbSerialNumber = null;
            SerialNumber localLynxChildUsbSerialNumber = null;
            SerialNumber localWebCamSerialNumber = null;
            RobotLog.ii("UsbScanner", "Scanning!");

//            if (Device.isRevControlHub()) {
//                localLynxParentUsbSerialNumber = LynxConstants.SERIAL_NUMBER_EMBEDDED;
//            }

            ScannedDevices devices = new HardwareDeviceManager(hardwareMap.appContext, getEventLoopManger().get()).scanForUsbDevices();
            for (SerialNumber sn : devices.keySet()) {
                DeviceManager.UsbDeviceType deviceType = devices.get(sn);
                if (deviceType == DeviceManager.UsbDeviceType.LYNX_USB_DEVICE) {
                    if(sn.isEmbedded()) {
                        if(Device.isRevControlHub()) {
                            if (localLynxParentUsbSerialNumber != null) {
                                throw new RuntimeException("Multiple Expansion Hubs (Lynx USB devices) are connected at once. Please disconnect the extra one(s)");
                            }
                        } else {
                            throw new RuntimeException("Embedded Lynx Device (Control Hub) found in not a control hub");
                        }

                        localLynxParentUsbSerialNumber = sn;
                    } else if(Device.isRevControlHub()) {
                        if (localLynxChildUsbSerialNumber != null) {
                            throw new RuntimeException("Multiple Expansion Hubs (Lynx USB devices) are connected at once. Please disconnect the extra one(s)");
                        }
                        localLynxChildUsbSerialNumber = sn;

                    } else {
                        if (localLynxParentUsbSerialNumber != null) {
                            throw new RuntimeException("Multiple Expansion Hubs (Lynx USB devices) are connected at once. Please disconnect the extra one(s)");
                        }
                        localLynxParentUsbSerialNumber = sn;
                    }

                } else if (deviceType == DeviceManager.UsbDeviceType.WEBCAM) {
                    if (localWebCamSerialNumber != null) {
                        throw new RuntimeException("Multiple Cameras are connected at once. Please disconnect the extra one(s)");
                    }
                    localWebCamSerialNumber = sn;
                }
            }

            if (localLynxParentUsbSerialNumber == null) {
                throw new RuntimeException("No Parent Lynx Module USB Device (Expansion / Control Hub) found. Please check that there is one connected");
            }

            if (localWebCamSerialNumber == null) {
                RobotLog.ww("UsbScanner", "No webcam found!\n" + Arrays.toString(Thread.currentThread().getStackTrace()));
                TeamCodeWarningSource.getInstance().setWarning("No webcam found!");
            } else {
                TeamCodeWarningSource.getInstance().clearWarning();
            }

            lynxParentUsbSerialNumber = localLynxParentUsbSerialNumber;
            lynxChildUsbSerialNumber = localLynxChildUsbSerialNumber;
            webCamSerialNumber = localWebCamSerialNumber;
        } catch (RobotCoreException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public boolean equals(Object obj) {
        if(obj instanceof UsbScanner) {
            UsbScanner other = (UsbScanner) obj;
            return other.lynxParentUsbSerialNumber.equals(lynxParentUsbSerialNumber) && (other.webCamSerialNumber == webCamSerialNumber || (other.webCamSerialNumber != null && webCamSerialNumber != null &&  other.webCamSerialNumber.equals(webCamSerialNumber)));
        }
        return false;
    }
}
