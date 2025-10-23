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
    public Score score;
    public Spike spike;
    public boolean park;
    public boolean useDistSensor;
    public Delay delay;

    private static class MotorData {
        private final DcMotorExEx motor;
        private final Field lastPos;
        private final SplitAutoRunner thiss;
        private final double power;
        private final double homingPower;

        public int getLastPos() {
            try {
                return (Integer) lastPos.get(thiss);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }

        public void setLastPos(int lastPos) {
            try {
                this.lastPos.set(thiss, lastPos);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }

        private MotorData(SplitAutoRunner thiss, DcMotorExEx motor, double power, double homingPower, Field lastPos) {
            this.motor = motor;
            this.lastPos = lastPos;
            this.lastPos.setAccessible(true);
            this.thiss = thiss;
            this.power = power;
            this.homingPower = homingPower;
        }
    }

    private MotorData getDriveFromMotor(Motor motor) {
        try {
            if(motor == Motor.SLIDE) {
                return new MotorData(
                        this,
                        hardware.intakeDrive,
                        MainBotConfig.MoveConfig.slidePower,
                        MainBotConfig.MoveConfig.slideHomingPower,
                        this.getClass().getDeclaredField("lastSlideCounts")
                );
            } else if(motor == Motor.PIVOT) {
                return new MotorData(
                        this,
                        hardware.intakeDrive,
                        MainBotConfig.MoveConfig.pivotPower,
                        MainBotConfig.MoveConfig.pivotHomingPower,
                        this.getClass().getDeclaredField("lastPivotCounts")
                );
            }

        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
        throw new RuntimeException("getDriveFromMotor() incomplete. Tried to get motor \"" + motor + "\"");
    }

    private Integer lastPivotCounts = new Integer(Integer.MAX_VALUE);
    private Integer lastSlideCounts = new Integer(Integer.MAX_VALUE);

    private void runMotorToPos(MotorData drive, int pos) {
        if(drive.getLastPos() != pos) {
            drive.motor.runMotorToPos(pos, drive.power);
            drive.setLastPos(pos);
        }
    }

    private void runMotorToPos(Motor motor, int pos) {
        runMotorToPos(getDriveFromMotor(motor), pos);
//        MainBotHardware.runMotorToPos(getDriveFromMotor(motor).motor, pos, getMotorPower(motor));
    }

    private void runMotorToPosBlocking(Motor motor, int targetPos, int endPos) {
        MotorData drive = getDriveFromMotor(motor);

        boolean wait = drive.getLastPos() != endPos;

        runMotorToPos(drive, targetPos);
//        MainBotHardware.runMotorToPos(drive.motor, pos, getMotorPower(motor));

        while (Math.abs(drive.motor.getCurrentPosition() - endPos) > doneThresh && opModeIsActive() && wait) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException ignored) {}
        }
    }

    @Override
    protected void init_internal() {
        optionDriver = new OptionDriver();
        try {
            optionDriver.add(new EnumOption<>("Alliance", "dpad_up", gamepad1, this.getClass().getField("alliance"), this, Alliance.class));
            optionDriver.add(new EnumOption<>("Right wheels", "dpad_down", gamepad1, this.getClass().getField("startPos"), this, StartPos.class));
            optionDriver.add(new EnumOption<>("Delay", "x", gamepad1, this.getClass().getField("delay"), this, Delay.class));
            optionDriver.add(new EnumOption<>("Score", "y", gamepad1, this.getClass().getField("score"), this, Score.class));
            optionDriver.add(new EnumOption<>("Spike", "dpad_right", gamepad1, this.getClass().getField("spike"), this, Spike.class));
            optionDriver.add(new BooleanOption("Park", "a", gamepad1, this.getClass().getField("park"), this));
            optionDriver.add(new BooleanOption("Use distance sensor", "b", gamepad1, this.getClass().getField("useDistSensor"), this));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        hardware = new MainBotHardware(hardwareMap);
        hardware.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        hardware.intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.intakeDrive.setPower(MainBotConfig.MoveConfig.slideHomingPower);
        hardware.intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.intakeDrive.setPower(MainBotConfig.MoveConfig.pivotHomingPower);

        splitAuto = new SplitAuto(
                (startPose) -> {
                    mecanumDrive = new MecanumDrive(hardware, startPose);
                    return mecanumDrive.actionBuilder(startPose);
                },
                (doubl) -> {
                    hardware.intakeServo.setPosition(doubl);
                },
                this::runMotorToPos,
                (motor, pos) -> {
                    runMotorToPosBlocking(motor, pos, pos);
                },
                this::runMotorToPosBlocking,
                (motor) -> {
                    MotorData drive = getDriveFromMotor(motor);
                    drive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    drive.motor.setPower(drive.homingPower);
                },
                () -> {
                    return null;
                }
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

            path = splitAuto.fillBuilder(delay, startPos, score, spike, park, useDistSensor).build();

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
        hardware.intakeDrive.setPower(MainBotConfig.MoveConfig.slidePower);
        hardware.intakeDrive.setTargetPosition(0);

        hardware.intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.intakeDrive.setPower(MainBotConfig.MoveConfig.pivotPower);
        hardware.intakeDrive.setTargetPosition(MainBotConfig.MoveConfig.restingPivotCounts);

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
