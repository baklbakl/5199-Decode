package org.ducktapeandaprayer.ftc.tele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.LynxModuleInitializer;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.IntOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.TripleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;

import java.util.ArrayList;

@TeleOp(name = "Turret Test", group = MainBotConfig.opmodeGroupName)
public class TurretTest extends FastIterativeOpMode {
    private MainBotHardware hardware = null;
    private ArrayList<VariableWatcher> variableWatchers = new ArrayList<>();

    protected void init_internal() {
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        hardware.modules.enableBulkCaching(LynxModule.BulkCachingMode.MANUAL);

        try {
            variableWatchers.add(new TripleVariableWatcher<>(gamepad1, Gamepad.class.getField("dpad_left"), Gamepad.class.getField("dpad_down"), Gamepad.class.getField("dpad_right"), (Boolean dpad_left, Boolean dpad_down, Boolean dpad_right) -> {
                //No three may be pressed at a time
                if(dpad_down ? (dpad_left || dpad_right) : (dpad_left && dpad_right)) {
                    return;
                }

                if(dpad_down) {
                    hardware.turretToHeading(0);
                } else if(dpad_left) {
                    hardware.turretToHeading(MainBotHardware.turretMaxHeading);
                } else if(dpad_right) {
                    hardware.turretToHeading(-MainBotHardware.turretMaxHeading);
                }
            }));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    protected void init_loop_internal() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    protected void start_internal() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    protected void loop_internal() {
        hardware.modules.clearBulkCache();
        for(VariableWatcher watcher : variableWatchers) {
            watcher.tick();
        }

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    protected void stop_internal() {
    }
}
