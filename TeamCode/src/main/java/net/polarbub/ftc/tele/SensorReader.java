package net.polarbub.ftc.tele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.polarbub.ftc.hardware.LynxModuleInitializer;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.lib.FastIterativeOpMode;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Sensor Reader", group = MainBotConfig.opmodeGroupName)
public class SensorReader extends FastIterativeOpMode {
    private final List<DcMotorEx> motors = new ArrayList<>();
    private final List<AnalogInput> analogInputs = new ArrayList<>();
    private final ElapsedTime runtime = new ElapsedTime();
    private LynxModuleInitializer hardware;

    protected void init_internal() {
        hardware = LynxModuleInitializer.getOrCreate(hardwareMap);

        for(int i = 0; i < LynxConstants.NUMBER_OF_MOTORS; i++) {
            motors.add(hardware.parentModule.createDcMotor(i));
        }

        if(hardware.childModule != null) {
            for(int i = 0; i < LynxConstants.NUMBER_OF_MOTORS; i++) {
                motors.add(hardware.childModule.createDcMotor(i));
            }
        }

        for(DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for(int i = 0; i < LynxConstants.NUMBER_OF_ANALOG_INPUTS; i++) {
            analogInputs.add(hardware.parentModule.createAnalogInput(i));
        }

        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        hardware.enableBulkCaching(LynxModule.BulkCachingMode.MANUAL);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    protected void loop_internal() {
        hardware.clearBulkCache();

        StringBuilder sb = new StringBuilder();
        sb.append("Run Time: ").append(runtime.toString()).append('\n');
        int i = 0;
        for(DcMotorEx m : motors) {
            sb.append("Encoder ");
            sb.append(i);
            sb.append(": ");
            sb.append(m.getCurrentPosition());
            sb.append("\n");
            i++;
        }

        i = 0;
        for(AnalogInput input : analogInputs) {
            sb.append("Analog ");
            sb.append(i);
            sb.append(": ");
            sb.append(input.getVoltage());
            sb.append("V\n");
            i++;
        }

        telemetry.addLine(sb.toString());
        telemetry.update();
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
