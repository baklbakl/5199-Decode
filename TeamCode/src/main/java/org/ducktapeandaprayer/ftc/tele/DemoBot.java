package org.ducktapeandaprayer.ftc.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.LynxModuleInitializer;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="DemoBot (Small)", group = MainBotConfig.demoGroupName)
public class DemoBot extends FastIterativeOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private List<VariableWatcher> variablesWatched = null;

    @Override
    protected void init_internal() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LynxModuleInitializer hardware = LynxModuleInitializer.getOrCreate(hardwareMap);

        leftDrive = hardware.parentModule.createDcMotor(0);
        rightDrive = hardware.parentModule.createDcMotor(3);

        variablesWatched = new ArrayList<>();
        try {
            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getDeclaredField("left_stick_y"), Gamepad.class.getDeclaredField("right_stick_x"), Float::equals,
                    (Float left_stick_y, Float right_stick_x) -> {
                        double drive = -left_stick_y / 2;
                        double turn = -right_stick_x / 2;
                        if(turn != 0) {
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

    }

    @Override
    protected void start_internal() {

    }

    @Override
    protected void loop_internal() {
        for(VariableWatcher watcher : variablesWatched) {
            watcher.tick();
        }
    }

    @Override
    protected void stop_internal() {

    }
}
