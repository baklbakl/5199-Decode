package org.ducktapeandaprayer.ftc.tele;

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.LynxUsbDeviceImpl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.LynxModuleDescription;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.LynxDcMotorControllerEx;
import org.ducktapeandaprayer.ftc.hardware.LynxModuleInitializer;
import org.ducktapeandaprayer.ftc.hardware.UsbScanner;
import org.ducktapeandaprayer.ftc.lib.EnumToStringHelper;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.LoopTimeCalculator;
import org.ducktapeandaprayer.ftc.lib.OnCreateHandlers;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;

import static org.ducktapeandaprayer.ftc.lib.OnCreateHandlers.getEventLoopManger;

@TeleOp(name = "PacketRateTester", group = MainBotConfig.opmodeGroupName)
public class PacketRateTester extends FastIterativeOpMode {
    public enum LynxModules {
        CONTROL_HUB_INTERNAL("Control Hub Internal Lynx Module"),
        CONTROL_HUB_RS485("Lynx Module Connected to Control Hub with RS485"),
        EXPANSION_HUB_USB("Expansion Hub Connected with USB"),

        ;

        private final String string;

        @Override
        public String toString() {
            return string;
        }

        LynxModules(String string) {
            this.string = string;

        }
    }

    public enum PacketType {
        KEEP_ALIVE,
        BULK_READ,
        MOTOR_POWER,

        ;

        @Override
        public String toString() {
            return EnumToStringHelper.toStringHelper(super.toString());
        }
    }

    public LynxModules moduleTypeToTest;
    public PacketType packetType;
    private OptionDriver options;

    @Override
    protected void init_internal() {
        options = new OptionDriver();
        try {
            options.add(new EnumOption<>("Lynx Module", "triangle", gamepad1, this.getClass().getField("moduleTypeToTest"), this, LynxModules.class));
            options.add(new EnumOption<>("Packet Type", "square", gamepad1, this.getClass().getField("packetType"), this, PacketType.class));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    protected void init_loop_internal() {
        String out = options.tick();
        if(out != null) {
            telemetry.addLine(out);
            telemetry.update();
        }
    }

    private LynxModuleInitializer.LynxModuleControllers moduleToTest;
    private LoopTimeCalculator timeCalculator;
    private Method pingMethod;
    private Method setMotorPowerInternalMethod;

    @Override
    protected void start_internal() {
        timeCalculator = new LoopTimeCalculator(500);
        //FIX: Add back ability to select usb vs RS485
        //LynxModuleInitializer initializer = LynxModuleInitializer.getOrCreate(hardwareMap, moduleTypeToTest == LynxModules.CONTROL_HUB_RS485);
        LynxModuleInitializer initializer = LynxModuleInitializer.getOrCreate(hardwareMap);
        initializer.enableBulkCaching(LynxModule.BulkCachingMode.MANUAL);

        switch (moduleTypeToTest) {
            case CONTROL_HUB_INTERNAL:
                moduleToTest = initializer.parentModule;
                break;

            case CONTROL_HUB_RS485:
            case EXPANSION_HUB_USB:
                if(initializer.childModule == null) {
                    throw new RuntimeException("No module found for selected mode \"" + moduleTypeToTest + "\"");
                }

                moduleToTest = initializer.childModule;
                break;
        }

        try {
            Class<LynxModule> clazz = LynxModule.class;
            pingMethod = clazz.getDeclaredMethod("ping");
            pingMethod.setAccessible(true);

            setMotorPowerInternalMethod = LynxDcMotorController.class.getDeclaredMethod("internalSetMotorPower", int.class, double.class, boolean.class);
            setMotorPowerInternalMethod.setAccessible(true);

            Method m = clazz.getDeclaredMethod("stopPingTimer", boolean.class);
            m.setAccessible(true);
            m.invoke(moduleToTest.module, true);
        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
        timeCalculator.reset();
    }

    @Override
    protected void loop_internal() {
        moduleToTest.module.clearBulkCache();
        timeCalculator.tick();
        telemetry.addLine(timeCalculator.toString());
        telemetry.update();

        try {
            switch (packetType) {
                case KEEP_ALIVE:
                    pingMethod.invoke(moduleToTest.module);
                    break;
                case BULK_READ:
                    moduleToTest.module.getBulkData();
                    break;
                case MOTOR_POWER:
                    setMotorPowerInternalMethod.invoke(moduleToTest.motorController, 0, 0.0, true);
                    break;
            }


        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    protected void stop_internal() {

        try {
            Method m = LynxModule.class.getDeclaredMethod("startPingTimer");
            m.setAccessible(true);
            m.invoke(moduleToTest.module);
        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e) {
            throw new RuntimeException(e);
        }

    }
}
