package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.*;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.EnumOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "1ThreadedTele", group = MainBotConfig.opmodeGroupName)
public class ThreadedTele extends ThreadedOpMode {
    public MainBotHardware hardware = null;

    @Override
    protected void init_internal() {
        super.setAlliance(Alliance.lastAlliance);
        super.init_internal();
        hardware = MainBotHardware.getOrCreate(hardwareMap, Alliance.lastAlliance == null);
        Alliance.lastAlliance = null;

        ControlHubThread controlHubThread = new ControlHubThread(hardware, this, gamepad1, super.getOptionDriver());
        ExpansionHubThread expansionHubThread = new ExpansionHubThread(hardware, this, controlHubThread, gamepad1, super.getOptionDriver());
        super.getThreads().put("control hub thread", controlHubThread);
        super.getThreads().put("expansion hub thread", expansionHubThread);
    }
}
