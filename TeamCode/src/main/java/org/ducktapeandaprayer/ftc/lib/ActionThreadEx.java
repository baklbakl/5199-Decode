package org.ducktapeandaprayer.ftc.lib;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;

public class ActionThreadEx extends ThreadEx {
    private final FtcDashboard dash;
    private final Canvas canvas;
    private final Action action;
    private final OpMode opMode;

    private boolean runAgain = true;

    public ActionThreadEx(Action action, OpMode opMode) {
        this.action = action;
        this.opMode = opMode;
        //This is stolen from Actions.runBlocking(), but it is put here in a way that we can use our loop time calculations
        dash = FtcDashboard.getInstance();
        canvas = new Canvas();
        action.preview(canvas);
    }

    //@Override
    //public void run() {
    //    super.run();
    //}

    private static final Field telemetryPacketLogField;
    private static final Field telemetryPacketDataField;

    static {
        try {
            telemetryPacketLogField = TelemetryPacket.class.getDeclaredField("log");
            telemetryPacketLogField.setAccessible(true);
            telemetryPacketDataField = TelemetryPacket.class.getDeclaredField("data");
            telemetryPacketDataField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    public void tick() {
        if (!runAgain || Thread.currentThread().isInterrupted()) {
            opMode.requestOpModeStop();
        } else {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());
            runAgain = action.run(packet);
            dash.sendTelemetryPacket(packet);

            try {
                List<String> log = (List<String>) telemetryPacketLogField.get(packet);
                for(String logItem : log) {
                    telemetry.append(logItem).append('\n');
                }
                SortedMap<String, String> data = (SortedMap<String, String>) telemetryPacketDataField.get(packet);
                for(Map.Entry<String, String> dataItem : data.entrySet()) {
                    telemetry.append(dataItem.getKey()).append(": ").append(dataItem.getValue()).append('\n');
                }
                telemetryPacketDataField.get(packet);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
