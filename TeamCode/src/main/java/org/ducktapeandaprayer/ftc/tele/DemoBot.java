package org.ducktapeandaprayer.ftc.tele;

import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.Range;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.LynxModuleInitializer;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="DemoBot (Small)", group = MainBotConfig.demoGroupName)
public class DemoBot extends FastIterativeOpMode {
    public Boolean slow = true;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private List<VariableWatcher> variablesWatched = null;
    private OptionDriver optionDriver = null;

    @Override
    protected void init_internal() {
        LynxModuleInitializer hardware = LynxModuleInitializer.getOrCreate(hardwareMap);
        if(hardwareMap.voltageSensor.size() == 0 && !Device.isRevControlHub()) {
            try {
                hardwareMap.voltageSensor.put("Expansion Hub Voltage Sensor", new LynxVoltageSensor(hardwareMap.appContext, hardware.parentModule.module));
            } catch (RobotCoreException | InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        leftDrive = hardware.parentModule.createDcMotor(3);
        rightDrive = hardware.parentModule.createDcMotor(0);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        variablesWatched = new ArrayList<>();
        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new BooleanOption("Slow mode (for kids)", "triangle", gamepad1, this.getClass().getField("slow"), this));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getDeclaredField("left_stick_y"), Gamepad.class.getDeclaredField("right_stick_x"), Float::equals,
                    (Float left_stick_y, Float right_stick_x) -> {
                        double drive = -left_stick_y / (slow ? 2 : 1);
                        double turn = right_stick_x / (slow ? 2 : 1);
                        if(turn != 0 && slow) {
                            drive = 0;
                        }
                        double leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
                        double rightPower = Range.clip(drive - turn, -1.0, 1.0) ;

                        // Send calculated power to wheels
                        leftDrive.setPower(leftPower);
                        rightDrive.setPower(rightPower);
                    }
            ));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    protected void init_loop_internal() {
        String out = optionDriver.tick();
        if(out != null) {
            telemetry.addLine(out);
            telemetry.update();
        }
    }

    @Override
    protected void start_internal() {}

    @Override
    protected void loop_internal() {
        for(VariableWatcher watcher : variablesWatched) {
            watcher.tick();
        }
        telemetry.addLine("Telemetry to make battery voltage send");
        telemetry.update();
    }

    @Override
    protected void stop_internal() {

    }
}
