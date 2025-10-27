package net.polarbub.ftc.hardware;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopBase;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.*;
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
import net.polarbub.ftc.lib.SingletonHelper;
import net.polarbub.ftc.lib.TeamCodeWarningSource;

import java.lang.ref.WeakReference;
import java.lang.reflect.Field;

import static net.polarbub.ftc.lib.OnCreateHandlers.eventLoop;

public class LynxModuleInitializer {
    public static LynxModuleInitializer getOrCreate(HardwareMap hardwareMap) {
        return SingletonHelper.getOrCreate(LynxModuleInitializer.class, () -> new LynxModuleInitializer(hardwareMap));
    }

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

    private LynxModuleInitializer(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        // Empty the hardware map so no one tries to use it except for LynxModules and LynxUsbDevices because the
        // FTC Libs uses the hardware map the close those.
        for(HardwareDevice device : hardwareMap.getAll(HardwareDevice.class)) {
            String name = String.valueOf(hardwareMap.getNamesOf(device));
            if(!(device instanceof LynxUsbDevice)) {
                RobotLog.ii("LynxModuleInitializer", "Removing device called : \"" + name + "\" Class: " + device.getClass().getCanonicalName() + " from hardwaremap");
                hardwareMap.remove(name, device);
            } else {
                RobotLog.ii("LynxModuleInitializer", "Leaving device called: \"" + name + "\" Class: " + device.getClass().getCanonicalName() + " from hardwaremap");
            }
        }

        EventLoopManager loopManager = null;

        try {
            Field loopHandlerField = FtcEventLoopBase.class.getDeclaredField("ftcEventLoopHandler");
            loopHandlerField.setAccessible(true);
            FtcEventLoop loop = eventLoop.get();
            FtcEventLoopHandler loopHandler = (FtcEventLoopHandler) loopHandlerField.get(loop);
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
            int parentModuleAddress;
            String errorString;
            if(Device.isRevControlHub()) {
                parentModuleAddress = LynxConstants.CH_EMBEDDED_MODULE_ADDRESS;
                errorString = "Could not find built in parent Lynx Module inside Control Hub (something has gone seriously wrong). Address " + parentModuleAddress;
            } else {
                parentModuleAddress = MainBotConfig.expansionHubAddress;
                errorString = "Could not find parent USB connected Expansion Hub with address " + parentModuleAddress + ". Check the connection and that the address is set correctly.";
            }

            try {
                parentModule = new LynxModuleControllers(usbDevice.getOrAddModule((new LynxModuleDescription.Builder(parentModuleAddress, true)).build()));
                parentModule.module.pingAndQueryKnownInterfacesAndEtc();
            } catch (RobotCoreException e) {
                throw new RuntimeException(errorString, e);
            }

            //Put the control hub back so that stopping when the opmode works right
            hardwareMap.put("Control Hub " + parentModule.module, parentModule.module);

            TeamCodeWarningSource.getInstance().clearGlobalWarning();

            LynxModuleControllers localChildModule = null;
            if(Device.isRevControlHub()) {
                try {
                    localChildModule = new LynxModuleControllers(
                            usbDevice.getOrAddModule((new LynxModuleDescription.Builder(1, false)).build())
                    );
                    localChildModule.module.pingAndQueryKnownInterfacesAndEtc();
                    //Put the control hub back so that stopping when the opmode works right
                    hardwareMap.put("Expansion Hub " + localChildModule.module.toString(), localChildModule.module);
                    
                } catch (RobotCoreException e) {
                    if(localChildModule != null) {
                        localChildModule.module.close();
                    }

                    localChildModule = null;
                    TeamCodeWarningSource.getInstance().setGlobalWarning(
                            "Could not find RS485 connected Expansion Hub with address "
                            + MainBotConfig.expansionHubAddress
                            + ". This may be expected if no Expansion Hub is present. Check that the hub is connected, " +
                            "powered up, and that the address is correct."
                    );
                }
            }
            childModule = localChildModule;
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
}
