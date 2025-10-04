package net.polarbub.ftc.tele;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.SerialNumber;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.lib.optiondriver.BooleanOption;
import net.polarbub.ftc.lib.optiondriver.EnumOption;
import net.polarbub.ftc.lib.optiondriver.OptionDriver;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import org.firstinspires.ftc.robotcore.internal.usb.LynxModuleSerialNumber;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Run Motor", group = MainBotConfig.opmodeGroupName)
public class RunMotor extends FastIterativeOpMode {
    private DcMotorEx drive = null;
    private ElapsedTime runtime = new ElapsedTime();
    List<LynxModule> allHubs;

    public enum HUB {
        CONTROL,
        EXPANSION;

        @Override
        public String toString() {
            if(this == CONTROL) {
                return "Control Hub";
            } else if(this == EXPANSION) {
                return "Expansion Hub";
            } else {
                throw new RuntimeException("HUB toString() not finished");
            }
        }
    }

    public HUB hub = HUB.CONTROL;
    public boolean addTwo;
    public boolean addOne;

    private OptionDriver optionDriver;
    private LynxDcMotorController controlController;
    private LynxDcMotorController expansionController;

    protected void init_internal() {
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            try {
                if(module.isParent()) {
                    controlController = new LynxDcMotorController(hardwareMap.appContext, module);
                } else {
                    expansionController = new LynxDcMotorController(hardwareMap.appContext, module);
                }
            } catch (RobotCoreException | InterruptedException e) {
                throw new RuntimeException(e);
            }
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
        optionDriver = new OptionDriver();
        try {
            if(controlController != null && expansionController != null) {
                optionDriver.add(new EnumOption<>("Hub", "dpad_up", gamepad1, this.getClass().getField("hub"), this, HUB.class));
            }
            optionDriver.add(new BooleanOption("+2", "dpad_left", gamepad1, this.getClass().getField("addTwo"), this));
            optionDriver.add(new BooleanOption("+1", "dpad_right", gamepad1, this.getClass().getField("addOne"), this));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }



        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    protected void loop_internal() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        String out = optionDriver.tick();
        if(out != null) {
            telemetry.addLine(out);
            telemetry.update();

            int motor = 0;
//            if(hub == HUB.EXPANSION) motor += 4;
            if(addTwo) motor += 2;
            if(addOne) motor += 1;

            if(hub == HUB.CONTROL) {
                drive = new DcMotorImplEx(controlController, motor);
            } else if(hub == HUB.EXPANSION) {
                drive = new DcMotorImplEx(expansionController, motor);
            }
            drive.setPower(0);

//            drive = (DcMotorEx) hardwareMap.get(DcMotor.class, "M" + motor);
        }

        drive.setPower(gamepad1.left_trigger - gamepad1.right_trigger);


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
