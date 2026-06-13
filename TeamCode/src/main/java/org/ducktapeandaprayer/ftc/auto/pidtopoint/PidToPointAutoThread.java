package org.ducktapeandaprayer.ftc.auto.pidtopoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.ducktapeandaprayer.ftc.auto.RobotImplThread;
import org.ducktapeandaprayer.ftc.auto.SplitAuto;
import org.ducktapeandaprayer.ftc.auto.roadrunner.MecanumDrive;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.Alliance;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;
import org.ducktapeandaprayer.ftc.lib.optiondriver.IntOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.lang.Math;
import java.util.concurrent.atomic.AtomicBoolean;

public class PidToPointAutoThread extends ThreadEx {
    private final MainBotHardware hardware;
    private final MainBotHardware.Container container;
    private final ThreadedOpMode opMode;

    public int cycleCount;

    public PidToPointAutoThread(MainBotHardware hardware, ThreadedOpMode opMode, RobotImplThread robot) {
        this.hardware = hardware;
        this.opMode = opMode;

        this.container = this.hardware.getContainer();
        container.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        OptionDriver optionDriver = this.opMode.getOptionDriver();
        Gamepad gamepad1 = this.opMode.gamepad1;

        try {
            optionDriver.add(new IntOption("Cycle Count", "square", gamepad1, this.getClass().getField("cycleCount"), this, 0, 3));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void run() {
        Alliance.lastAlliance = opMode.getAlliance();
        super.run();
    }

    @Override
    public void tick() {
        container.modules.parentModule.module.clearBulkCache();


    }
}
