package net.polarbub.ftc.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import net.polarbub.ftc.hardware.DcMotorExEx;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.auto.options.*;
import net.polarbub.ftc.auto.roadrunner.MecanumDrive;
import net.polarbub.ftc.lib.Alliance;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import net.polarbub.ftc.lib.optiondriver.BooleanOption;
import net.polarbub.ftc.lib.optiondriver.EnumOption;
import net.polarbub.ftc.lib.optiondriver.OptionDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Field;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Split Auto")
public class SplitAutoRunner extends FastIterativeOpMode {
    private final int doneThresh = 20;

    private OptionDriver optionDriver;
    private MainBotHardware hardware;
    private SplitAuto splitAuto;
    private Action path = null;
    private MecanumDrive mecanumDrive;
//    private double lastUpdateTime

    public Alliance alliance;
    public StartPos startPos;
    public Park park;
    public Delay delay;

    private static class MotorData {
        private final DcMotorExEx motor;
        private final SplitAutoRunner thiss;
        private final double power;



        private MotorData(SplitAutoRunner thiss, DcMotorExEx motor, double power) {
            this.motor = motor;
            this.thiss = thiss;
            this.power = power;
        }
    }

    private MotorData getDriveFromMotor(Motor motor) {
        if(motor == Motor.INTAKE) {
            return new MotorData(
                    this,
                    hardware.intakeDrive,
                    MainBotConfig.MoveConfig.intakeDrivePower
            );
        } else if(motor == Motor.LAUNCHER_ONE) {
            return new MotorData(
                    this,
                    hardware.launchDrive,
                   MainBotConfig.MoveConfig.launchDrivePower
            );
        }  else if(motor == Motor.LAUNCHER_TWO) {
            return new MotorData(
                    this,
                    hardware.launchDrive2,
                    MainBotConfig.MoveConfig.launchDrivePower
            );
        }

        throw new RuntimeException("getDriveFromMotor() incomplete. Tried to get motor \"" + motor + "\"");
    }

    public void runMotor(Motor motor, double power) {
        runMotor(getDriveFromMotor(motor), power);
    }

    public void runMotor(MotorData drive, double power) {
        drive.motor.setPower(power);
    }

    public void runServo(double pos) {
        hardware.moverServo.setPosition(pos);
        hardware.moverServo2.setPosition(pos);
    }



    @Override
    protected void init_internal() {
        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new EnumOption<>("Alliance", "dpad_up", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
            optionDriver.add(new EnumOption<>("Start Pos", "dpad_down", gamepad1, this.getClass().getField("startPos"), this, StartPos.class));
            optionDriver.add(new EnumOption<>("Delay", "x", gamepad1, this.getClass().getField("delay"), this, Delay.class));
            optionDriver.add(new BooleanOption("Park", "a", gamepad1, this.getClass().getField("park"), this));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        hardware = MainBotHardware.getOrCreate(hardwareMap);
        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        hardware.intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hardware.intakeDrive.setPower(MainBotConfig.MoveConfig.slideHomingPower);
        hardware.intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hardware.intakeDrive.setPower(MainBotConfig.MoveConfig.pivotHomingPower);

        splitAuto = new SplitAuto(
                (startPose) -> {
                    mecanumDrive = new MecanumDrive(hardware, startPose);
                    return mecanumDrive.actionBuilder(startPose);
                },
                (motor, power) -> {
                    runMotor(motor, power);
                },
                this::runServo
        );
    }

    @Override
    protected void init_loop_internal() {
        String out = optionDriver.tick();
        if(out != null) {
            telemetry.addLine(out);
            telemetry.update();

            if(alliance == Alliance.BLUE) {
                hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if (alliance == Alliance.RED) {
                hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }

            path = splitAuto.fillBuilder(delay, startPos, park).build();

            telemetry.addLine(out);
            telemetry.addLine("\nDone building paths");
            telemetry.update();
        }
        try {
            Thread.sleep(1);
        } catch (InterruptedException ignored) {}
    }

    @Override
    protected void start_internal() {
        Alliance.lastAlliance = alliance;

        hardware.intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        hardware.intakeDrive.setPower(MainBotConfig.MoveConfig.slidePower);
        hardware.intakeDrive.setTargetPosition(0);

        hardware.intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        hardware.intakeDrive.setPower(MainBotConfig.MoveConfig.pivotPower);
//        hardware.intakeDrive.setTargetPosition(MainBotConfig.MoveConfig.restingPivotCounts);

        AtomicBoolean pathDone = new AtomicBoolean(false);
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                path,
                                new InstantAction(() -> {
                                    pathDone.set(true);
                                })

                        ),
                        (TelemetryPacket telemetryPacket) -> {
                            telemetry.update();
                            return !pathDone.get();
                        }
                )
        );
        pathDone.set(true);
    }

    @Override
    protected void loop_internal() {
        telemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        telemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
        telemetry.update();
    }


    protected void stop_internal() {
        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}
