package net.polarbub.ftc.tele;

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.ScannedDevices;
import com.qualcomm.robotcore.util.SerialNumber;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.UsbScanner;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import net.polarbub.ftc.lib.OnCreateHandlers;

import java.util.List;
import java.util.Map;

import static net.polarbub.ftc.lib.OnCreateHandlers.getEventLoopManger;

@TeleOp(name = "PacketRateTester", group = MainBotConfig.opmodeGroupName)
public class PacketRateTester extends FastIterativeOpMode {
    private UsbScanner scanner = null;
    private List<LynxModule> modules = null;

    @Override
    protected void init_internal() {

        try {
            ScannedDevices devices = new HardwareDeviceManager(hardwareMap.appContext, OnCreateHandlers.getEventLoopManger().get()).scanForUsbDevices();
            devices.entrySet().stream().map((Map.Entry<SerialNumber, DeviceManager.UsbDeviceType> device) -> {
                return device;
            });

            for (Map.Entry<SerialNumber, DeviceManager.UsbDeviceType> device : devices.entrySet()) {
                if (device.getValue() == DeviceManager.UsbDeviceType.LYNX_USB_DEVICE) {

                }
            }
        } catch (RobotCoreException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    protected void init_loop_internal() {

    }

    @Override
    protected void start_internal() {

    }

    @Override
    protected void loop_internal() {

    }

    @Override
    protected void stop_internal() {

    }
}
