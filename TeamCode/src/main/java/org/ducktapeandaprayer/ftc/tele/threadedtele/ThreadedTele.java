package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.*;

@TeleOp(name = "1ThreadedTele", group = MainBotConfig.opmodeGroupName)
public class ThreadedTele extends ThreadedOpMode {
    public MainBotHardware hardware = null;

    @Override
    protected void init_internal() {
        super.setAlliance(Alliance.lastAlliance);
        super.init_internal();
        hardware = MainBotHardware.getOrCreate(hardwareMap, Alliance.lastAlliance == null);
        Alliance.lastAlliance = null;

        ControlHubThread controlHubThread = new ControlHubThread(hardware, this);
        ExpansionHubThread expansionHubThread = new ExpansionHubThread(hardware, this, controlHubThread);
        super.getThreads().put("control hub thread", controlHubThread);
        super.getThreads().put("expansion hub thread", expansionHubThread);
    }
}
