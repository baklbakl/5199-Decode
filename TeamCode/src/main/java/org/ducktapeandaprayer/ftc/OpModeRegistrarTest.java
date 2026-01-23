package org.ducktapeandaprayer.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public class OpModeRegistrarTest {
    public static class OpModeOne extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {}
    }

    public static class OpModeTwo extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {}
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        manager.register(
                new OpModeMeta.Builder()
                        .setName("Test OpMode One")
                        .setGroup("Test Group")
                        .setFlavor(OpModeMeta.Flavor.TELEOP)
                        .setSource(OpModeMeta.Source.ANDROID_STUDIO)
                        .build(),
                OpModeOne.class
        );

        manager.register(
                new OpModeMeta.Builder()
                        .setName("Test OpMode Two")
                        .setGroup("Test Group")
                        .setFlavor(OpModeMeta.Flavor.TELEOP)
                        .setSource(OpModeMeta.Source.ANDROID_STUDIO)
                        .build(),
                new OpModeTwo()
        );
    }
}
