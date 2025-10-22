package net.polarbub.ftc.hardware;

import android.content.Context;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopBase;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.hardware.lynx.commands.LynxInterface;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import net.polarbub.ftc.MainBotConfig;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

import java.lang.reflect.Field;
import java.util.List;

public class LynxModuleInitializer {
    private static FtcEventLoop eventLoop = null;
    public final HardwareMap hardwareMap;
    public final LynxModuleControllers parentModule;
    public final LynxModuleControllers childModule;

    public class LynxModuleControllers {
        public final LynxModule module;
        public final LynxDcMotorController motorController;
        public final LynxServoController servoController;
        public final LynxAnalogInputController analogInputController;
        public final LynxVoltageSensor voltageSensor;

        public LynxModuleControllers(LynxModule module) {
            this.module = module;
            try {
                motorController = new LynxDcMotorController(hardwareMap.appContext, module);
                servoController = new LynxServoController(hardwareMap.appContext, module);
                analogInputController = new LynxAnalogInputController(hardwareMap.appContext, module);
                voltageSensor = new LynxVoltageSensor(hardwareMap.appContext, module);
            } catch (RobotCoreException | InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        public DcMotorExEx createDcMotor(int id) {
            DcMotorExEx motor;

            motor = new DcMotorExEx(motorController, id, DcMotorSimple.Direction.FORWARD, MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
            motor.setTargetPositionTolerance(MainBotConfig.motorPositionTolerance);
            return motor;
        }

        public ServoImplEx createServo(int id) {
            return new ServoImplEx(servoController, id, ServoConfigurationType.getStandardServoType());
        }


        public AnalogInput createAnalogInput(int id) {
            return new AnalogInput(analogInputController, id);
        }

        public Encoder createEncoder(int id) {
            return new OverflowEncoder(new RawEncoder(createDcMotor(id)));
        }
    }

    public LynxModuleInitializer(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        // Empty the hardware map to close any of the other references to the LynxModules.
        for(String name : hardwareMap.getAllNames(HardwareDevice.class)) {
            HardwareDevice device = hardwareMap.get(name);
            if(device instanceof LynxModule) {
                device.close();
            }
            hardwareMap.remove(name, hardwareMap.get(name));

        }

        EventLoopManager loopManager = null;

        try {
            Field loopHandlerField = FtcEventLoopBase.class.getDeclaredField("ftcEventLoopHandler");
            loopHandlerField.setAccessible(true);
            FtcEventLoopHandler loopHandler = (FtcEventLoopHandler) loopHandlerField.get(eventLoop);
            loopManager = loopHandler.getEventLoopManager();
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException("MainBotHardware HardwareMap oopsy", e);
        }


        try {
            SerialNumber lynxSerialNumber = null;
            if(Device.isRevControlHub()) {
                lynxSerialNumber = LynxConstants.SERIAL_NUMBER_EMBEDDED;
            } else {
                //Scan for devices if we aren't connected to a control hub
                ScannedDevices devices = new HardwareDeviceManager(hardwareMap.appContext, loopManager).scanForUsbDevices();
                for(SerialNumber sn : devices.keySet()) {
                    if(devices.get(sn) == DeviceManager.UsbDeviceType.LYNX_USB_DEVICE) {
                        if(lynxSerialNumber != null) {
                            throw new RuntimeException("Multiple Expansion Hubs (Lynx USB devices) are connected at once. Please disconnect the extra one(s)");
                        }
                        lynxSerialNumber = sn;
                    }
                }

                if(lynxSerialNumber == null) {
                    throw new RuntimeException("No Expansion Hub (Lynx USB device) found. Please check that there is a Expansion Hub connected");
                }
            }

            LynxUsbDevice usbDevice = LynxUsbDeviceImpl.findOrCreateAndArm(hardwareMap.appContext, lynxSerialNumber, loopManager, HardwareDeviceManager.createUsbManager());

            if(Device.isRevControlHub()) {
                parentModule = new LynxModuleControllers(usbDevice.getOrAddModule((new LynxModuleDescription.Builder(LynxConstants.CH_EMBEDDED_MODULE_ADDRESS, true)).build()));
                childModule = new LynxModuleControllers(usbDevice.getOrAddModule((new LynxModuleDescription.Builder(1, false)).build()));
            } else {
                parentModule = new LynxModuleControllers(usbDevice.getOrAddModule((new LynxModuleDescription.Builder(1, true)).build()));
                childModule = null;
            }
        } catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void enableBulkCaching(LynxModule.BulkCachingMode mode) {
        parentModule.module.setBulkCachingMode(mode);
        childModule.module.setBulkCachingMode(mode);
    }

    public void clearBulkCache() {
        parentModule.module.clearBulkCache();
        childModule.module.clearBulkCache();
    }

    @OnCreateEventLoop
    public static void collectEventLoop(Context context, FtcEventLoop loop) {
        eventLoop = loop;
    }
}
