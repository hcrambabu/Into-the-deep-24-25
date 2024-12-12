package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name = "Concept: FirstKotlin", group = "Concept")
@Disabled
class FirstKotlinOp: LinearOpMode() {

    val runtime: ElapsedTime = ElapsedTime()
    lateinit var dcMotor: DcMotor


    fun initialize() {
        telemetry.addData("Staus", "FirstKotlinOp Initialized")
        dcMotor = hardwareMap.get(DcMotor::class.java, "leftFront")
        telemetry.update();
    }

    @OptIn(ExperimentalStdlibApi::class)
    fun runInLoop() {
        telemetry.addData("Status", "Runtime: " + runtime.toString())
        telemetry.addData("left_stick_y", "Value: " + gamepad1.left_stick_y)
        dcMotor.setPower(gamepad1.left_stick_y.toDouble());

        telemetry.update()
    }

    override fun runOpMode() {
        initialize()
        // Wait for the game to start (driver presses PLAY)
        waitForStart()
        runtime.reset()
        while (opModeIsActive()) {
            runInLoop()
        }
    }
}