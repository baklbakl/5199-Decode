package net.polarbub.ftc.tele;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.LynxModuleInitializer;
import net.polarbub.ftc.lib.optiondriver.EnumOption;
import net.polarbub.ftc.lib.optiondriver.IntOption;
import net.polarbub.ftc.lib.optiondriver.OptionDriver;
import net.polarbub.ftc.lib.FastIterativeOpMode;

@TeleOp(name = "Run Motor", group = MainBotConfig.opmodeGroupName)
public class RunMotor extends FastIterativeOpMode {
    private DcMotorEx drive = null;
    private ElapsedTime runtime = new ElapsedTime();

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
    public int motor = 0;

    private OptionDriver optionDriver;
    private LynxModuleInitializer hardware = null;

    protected void init_internal() {
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        hardware = LynxModuleInitializer.getOrCreate(hardwareMap);
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
            if(hardware.childModule != null) {
                optionDriver.add(new EnumOption<>("Hub", "dpad_up", gamepad1, this.getClass().getField("hub"), this, HUB.class));
            }
            optionDriver.add(new IntOption("Motor", "dpad_down", gamepad1, this.getClass().getField("motor"), this, 0, LynxConstants.NUMBER_OF_MOTORS - 1));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    protected void loop_internal() {
        hardware.clearBulkCache();

        String out = optionDriver.tick();
        if(out != null) {
            telemetry.addLine(out);
            telemetry.update();

            if(hub == HUB.CONTROL) {
                drive = hardware.parentModule.createDcMotor(motor);
            } else if(hub == HUB.EXPANSION) {
                drive = hardware.childModule.createDcMotor(motor);
            }
            drive.setPower(0);
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
