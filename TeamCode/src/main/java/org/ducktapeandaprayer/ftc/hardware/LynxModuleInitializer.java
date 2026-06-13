package org.ducktapeandaprayer.ftc.hardware;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.LastKnown;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.lib.SingletonHelper;
import org.ducktapeandaprayer.ftc.lib.TeamCodeWarningSource;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import static org.ducktapeandaprayer.ftc.lib.OnCreateHandlers.*;

public class LynxModuleInitializer {
    private final double newLastKnownTime = 3000;

    public enum UseRS485 {
        FORCE,
        FALLBACK,
        NEVER
    }

    private final UseRS485 useRS485;

    public static LynxModuleInitializer getOrCreate(HardwareMap hardwareMap, UseRS485 useRS485) {
        LynxModuleInitializer ts = SingletonHelper.getOrCreate(LynxModuleInitializer.class, () -> new LynxModuleInitializer(hardwareMap, useRS485));

        if(!ts.scanner.equals(UsbScanner.getOrCreate(hardwareMap)) || ts.useRS485 != useRS485) {
            SingletonHelper.clear(LynxModuleInitializer.class);
        }

        return SingletonHelper.getOrCreate(LynxModuleInitializer.class, () -> new LynxModuleInitializer(hardwareMap, useRS485));
    }


    public static LynxModuleInitializer getOrCreate(HardwareMap hardwareMap) {
        return getOrCreate(hardwareMap, UseRS485.FALLBACK);
    }

    public static void clearInstance() {
        SingletonHelper.clear(LynxModuleInitializer.class);
    }

    public final HardwareMap hardwareMap;
    public final LynxModuleControllers parentModule;
    public final LynxModuleControllers childModule;
    private final UsbScanner scanner;

    private static final Field msFreshnessField;
    private static final Field moduleControllersField;

    static {
        try {
            msFreshnessField = LastKnown.class.getDeclaredField("msFreshness");
            msFreshnessField.setAccessible(true);
            moduleControllersField = LynxModule.class.getDeclaredField("controllers");
            moduleControllersField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    public class LynxModuleControllers {
        public final LynxModule module;
        public final LynxDcMotorControllerEx motorController;
        public final LynxDigitalChannelController digitalController;
        public final LynxServoController servoController;
        public final LynxAnalogInputController analogInputController;
        public final LynxVoltageSensor voltageSensor;


        private void fixupLastKnown(LastKnown<?> lastKnown) {
            try {
                msFreshnessField.setDouble(lastKnown, newLastKnownTime);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }

        public LynxModuleControllers(LynxModule module) {
            this.module = module;
            try {
                motorController = new LynxDcMotorControllerEx(hardwareMap.appContext, module);
                servoController = new LynxServoController(hardwareMap.appContext, module);
                analogInputController = new LynxAnalogInputController(hardwareMap.appContext, module);
                voltageSensor = new LynxVoltageSensor(hardwareMap.appContext, module);
                digitalController = new LynxDigitalChannelController(hardwareMap.appContext, module);
            } catch (RobotCoreException | InterruptedException e) {
                throw new RuntimeException(e);
            }

            //This fixes up all the last known values in the controllers to be a larger, more sensible number
            try {
                List<LynxController> moduleControllers = (List<LynxController>) moduleControllersField.get(this.module);

                for(LynxController controller : moduleControllers) {
                    Field[] controllerFields = controller.getClass().getDeclaredFields();
                    for(Field controlerField : controllerFields) {
                        controlerField.setAccessible(true);
                        Object object = controlerField.get(controller);
                        if(object instanceof LastKnown[]) {
                            LastKnown<?>[] lastKnowns = (LastKnown<?>[]) object;
                            for(LastKnown<?> lastKnown : lastKnowns) {
                                fixupLastKnown(lastKnown);
                            }
                        } else if(object instanceof LastKnown) {
                            fixupLastKnown((LastKnown<?>) object);
                        }
                    }
                }
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }

        }

        public DcMotorExEx createDcMotor(int id) {
            DcMotorExEx motor;

            motor = new DcMotorExEx(motorController, id, DcMotorSimple.Direction.FORWARD, MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
            motor.setPower(0);
            return motor;
        }

        public ServoImplEx createServo(int id) {
            return new ServoImplEx(servoController, id, ServoConfigurationType.getStandardServoType());
        }

        public AnalogInput createAnalogInput(int id) {
            return new AnalogInput(analogInputController, id);
        }

        public DigitalChannel createDigitalChannel(int id) {
            return new DigitalChannelImpl(digitalController, id);
        }

        public Encoder createEncoder(int id) {
            return new OverflowEncoder(new RawEncoder(createDcMotor(id)));
        }
    }

    private LynxModuleControllers createChildModule(LynxUsbDevice parentUsbDevice) {
        LynxUsbDevice childUsbDevice = null;

        if(useRS485 == UseRS485.FORCE) {
            childUsbDevice = parentUsbDevice;
        } else {
            if(scanner.lynxChildUsbSerialNumber != null) {
                try {
                    childUsbDevice = LynxUsbDeviceImpl.findOrCreateAndArm(hardwareMap.appContext, scanner.lynxChildUsbSerialNumber, getEventLoopManger().get(), HardwareDeviceManager.createUsbManager());
                } catch (RobotCoreException | InterruptedException e) {
                    throw new RuntimeException(e);
                }
            } else if(useRS485 == UseRS485.FALLBACK) {
                childUsbDevice = parentUsbDevice;
            } else if(useRS485 == UseRS485.NEVER) {
                TeamCodeWarningSource.getInstance().setLynxModuleInitializerWarning(
                        "Could not find USB Connected Expansion Hub with address "
                                + MainBotHardware.expansionHubAddress
                                + " and RS485 fallback is disabled. Check that the module is powered on, "
                                + "connected, and that the address is correct"
                );
                return null;
            }
        }

        LynxModuleControllers localChildModule = null;
        try {
            localChildModule = new LynxModuleControllers(
                    childUsbDevice.getOrAddModule((new LynxModuleDescription.Builder(MainBotHardware.expansionHubAddress, childUsbDevice != parentUsbDevice)).build())
            );

            localChildModule.module.pingAndQueryKnownInterfacesAndEtc();
            //Put the module in the hardware map that stopping when the opmode works right
            hardwareMap.put("Expansion Hub " + localChildModule.module.toString(), localChildModule.module);

            if(childUsbDevice == parentUsbDevice) {
                if(useRS485 == UseRS485.FALLBACK) {
                    TeamCodeWarningSource.getInstance().setLynxModuleInitializerWarning(
                            "Could not find USB Connected Expansion Hub. " +
                                    "Falling back to RS485. This may be expected if no USB connected expansion Hub is present."
                    );
                }
            } else {
                //Put the usb device in the hardware that disconnections and reconnections work right. This means that there will be issues if a new eh it installed without a power cycle.
                hardwareMap.put("Expansion Hub Portal Created" + childUsbDevice.toString(), childUsbDevice);
            }

        } catch (RobotCoreException e) {
            RobotLog.logStackTrace("LynxModuleInitializer", e);

            if(localChildModule != null) {
                localChildModule.module.close();
            }

            if(parentUsbDevice == childUsbDevice) {
                TeamCodeWarningSource.getInstance().setLynxModuleInitializerWarning(
                        "Could not find Expansion Hub with address "
                                + MainBotHardware.expansionHubAddress
                                + " connected via RS485" + (useRS485 == UseRS485.FALLBACK ? " or USB" : "") + ". Check that the module is powered on, "
                                + "connected, and that the address is correct"
                );
            } else {
                //Only close the child usb device if it is distinct from the parent one. When we are not connected with RS485
                childUsbDevice.close();

                TeamCodeWarningSource.getInstance().setLynxModuleInitializerWarning(
                        "Could not find Expansion Hub with address "
                                + MainBotHardware.expansionHubAddress
                                + " connected via USB. This is strange because a USB device was found, "
                                + "but the lynx module itself was not. Check the address of the module is correct"
                );
            }
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        return localChildModule;
    }

    private LynxModuleInitializer(HardwareMap hardwareMap, UseRS485 useRS485) {
        this.hardwareMap = hardwareMap;
        this.useRS485 = useRS485;

        RobotLog.ii("LynxModuleInitializer", "Initializing modules");
        // Empty the hardware map so no one tries to use it except for LynxModules and LynxUsbDevices because the
        // FTC Libs uses the hardware map the close them.
        for(HardwareDevice device : hardwareMap.getAll(HardwareDevice.class)) {
            String name = String.valueOf(hardwareMap.getNamesOf(device));
            if(device instanceof LynxUsbDevice || device instanceof VoltageSensor) {
                RobotLog.ii("LynxModuleInitializer", "Leaving device called: \"" + name + "\" Class: " + device.getClass().getCanonicalName() + " from hardwaremap");
            } else {
                RobotLog.ii("LynxModuleInitializer", "Removing device called : \"" + name + "\" Class: " + device.getClass().getCanonicalName() + " from hardwaremap");
                hardwareMap.remove(name, device);
                if(!(device instanceof LynxModule)) {
                    device.close();
                }
            }
        }

        //Empty out device specific map
        for(HardwareMap.DeviceMapping<? extends HardwareDevice> mapping : hardwareMap.allDeviceMappings) {
            Set<? extends Map.Entry<String, ? extends HardwareDevice>> entries = mapping.entrySet();
            Optional<? extends Map.Entry<String, ? extends HardwareDevice>> entry = entries.stream().findAny();

            if(entry.isPresent()) {
                if(entry.get().getValue() instanceof VoltageSensor) {
                    continue;
                }

                for(Map.Entry<String, ? extends HardwareDevice> device : entries) {
                    mapping.remove(device.getKey());
                    if(!(device.getValue() instanceof LynxModule)) {
                        device.getValue().close();
                    }
                }
            }
        }

        try {
            scanner = UsbScanner.getOrCreate(hardwareMap);

            LynxUsbDevice parentUsbDevice = LynxUsbDeviceImpl.findOrCreateAndArm(hardwareMap.appContext, scanner.lynxParentUsbSerialNumber, getEventLoopManger().get(), HardwareDeviceManager.createUsbManager());
            int parentModuleAddress;
            String errorString;
            if(Device.isRevControlHub()) {
                parentModuleAddress = LynxConstants.CH_EMBEDDED_MODULE_ADDRESS;
                errorString = "Could not find built in parent Lynx Module inside Control Hub (something has gone seriously wrong). Address " + parentModuleAddress;
            } else {
                parentModuleAddress = MainBotHardware.expansionHubAddress;
                errorString = "Could not find parent USB connected Expansion Hub with address " + parentModuleAddress + ". Check the connection and that the address is set correctly.";
            }

            try {
                parentModule = new LynxModuleControllers(parentUsbDevice.getOrAddModule((new LynxModuleDescription.Builder(parentModuleAddress, true)).build()));
                parentModule.module.pingAndQueryKnownInterfacesAndEtc();
            } catch (RobotCoreException e) {
                throw new RuntimeException(errorString, e);
            }

            //Put the control hub back so that stopping when the opmode works right
            hardwareMap.put("Control Hub " + parentModule.module, parentModule.module);

            TeamCodeWarningSource.getInstance().clearLynxModuleInitializerWarning();

            LynxModuleControllers localChildModule = null;
            if(Device.isRevControlHub()) {
                localChildModule = createChildModule(parentUsbDevice);
            }
            childModule = localChildModule;
        } catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void enableBulkCaching(LynxModule.BulkCachingMode mode) {
        parentModule.module.setBulkCachingMode(mode);
        if(childModule != null) {
            childModule.module.setBulkCachingMode(mode);
        }

    }

    public void clearBulkCache() {
        parentModule.module.clearBulkCache();
        if(childModule != null) {
            childModule.module.clearBulkCache();
        }
    }
}
