package org.ducktapeandaprayer.ftc.tele.threadedtele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.*;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;

@TeleOp(name = "1ThreadedTele", group = MainBotConfig.opmodeGroupName)
public class ThreadedTele extends ThreadedOpMode {
    public MainBotHardware hardware = null;

    @Override
    protected void init_internal() {
        super.setAlliance(Alliance.lastAlliance);
        super.init_internal();
        boolean resetOctoQuad = Alliance.lastAlliance == null;
        hardware = MainBotHardware.getOrCreate(hardwareMap, resetOctoQuad);
        Alliance.lastAlliance = null;
        ControlHubThread controlHubThread = new ControlHubThread(hardware, this, resetOctoQuad);
        ExpansionHubThread expansionHubThread = new ExpansionHubThread(hardware, this, controlHubThread);
        super.getThreads().put("control hub thread", controlHubThread);
        super.getThreads().put("expansion hub thread", expansionHubThread);
    }

    @Override
    protected void start_internal() {
        hardware.getContainer();
        super.start_internal();
    }
}
