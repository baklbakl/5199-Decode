package net.polarbub.ftc.hardware;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

import static java.lang.Double.NaN;

//We add a few helper functions to DcMotorImplExS
public class DcMotorExEx extends DcMotorImplEx {
    private final LynxDcMotorControllerEx controller;

    public DcMotorExEx(LynxDcMotorControllerEx controller, int portNumber) {
        super(controller, portNumber);
        this.controller = controller;
    }

    public DcMotorExEx(LynxDcMotorControllerEx controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
        this.controller = controller;
    }

    public DcMotorExEx(LynxDcMotorControllerEx controller, int portNumber, Direction direction, @NonNull @NotNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
        this.controller = controller;
    }

    public void runMotorToPos(int targetPos, double power) {
        this.setTargetPosition(targetPos);
        //This is cached for us, so it won't be updated if the power we are setting to is the same as the current one
        if(!Double.isNaN(power)) this.setPower(power);
        //This is cached for us, so it won't be updated if we are already in run_to_position
        this.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runMotorToPos(int targetPos) {
        this.runMotorToPos(targetPos, NaN);
    }

    public double getTargetVelocity() {
        return controller.getTargetMotorVelocity(getPortNumber());
    }
}
