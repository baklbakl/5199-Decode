package org.ducktapeandaprayer.ftc.tele

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.AsymMotorFeedforward
import com.acmerobotics.roadrunner.TestingMotorFeedforward
import com.acmerobotics.roadrunner.TimeProfile
import com.acmerobotics.roadrunner.constantProfile
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.ducktapeandaprayer.ftc.MainBotConfig
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware

@TeleOp(group = MainBotConfig.opmodeGroupName)
class LauncherFeedForwardTester : LinearOpMode() {
    companion object {
        @JvmField
        var DISTANCE = 13000.0
        @JvmField
        var kASpeedUp = 0.0105
        @JvmField
        var kASlowDown = 0.0
        @JvmField
        var kV = 0.0052
        @JvmField
        var kS = 0.0
        @JvmField
        var maxVel: Double = 1500.0
        @JvmField
        var minAccel: Double = -400.0
        @JvmField
        var maxAccel: Double = 400.0
        @JvmField
        var negDec: Double = 0.0
        @JvmField
        var brake = DcMotor.ZeroPowerBehavior.BRAKE
    }

    override fun runOpMode() {
        val hardware = MainBotHardware.getOrCreate(hardwareMap)
        hardware.launchDrive.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hardware.launchDrive.zeroPowerBehavior = brake;

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val profile = TimeProfile(constantProfile(
            DISTANCE, 0.0, maxVel, minAccel, maxAccel).baseProfile)


        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()

        waitForStart()

        if (isStopRequested) return

        var movingForwards = true
        var startTs = System.nanoTime() / 1e9

        while (!isStopRequested) {
            telemetry.addData("v", hardware.launchDrive.getVelocity())

            val ts = System.nanoTime() / 1e9
            val t = ts - startTs
            if (t > profile.duration) {
                movingForwards = !movingForwards
                startTs = ts
            }

            var v = profile[t].drop(1)
            if (!movingForwards) {
                v = v.unaryMinus()
            }

            telemetry.addData("vref", v[0])

            val power = TestingMotorFeedforward(kS, kV, kASpeedUp, kASlowDown, negDec).compute(v) / hardware.parentVoltageSensor.voltage
            telemetry.addData("power", power * maxVel / 2)
            hardware.launchDrive.power = power

            telemetry.addData("min", -maxVel)
            telemetry.addData("max", maxVel)
            telemetry.update()
        }
    }
}