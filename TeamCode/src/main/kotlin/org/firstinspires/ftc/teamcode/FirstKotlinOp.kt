package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.hardware.rev.RevTouchSensor
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
class FirstKotlinOp: LinearOpMode() {

    val runtime: ElapsedTime = ElapsedTime()
    lateinit var colorSensor: RevColorSensorV3
    lateinit var dcMotor: DcMotor
    lateinit var imu: IMU

    fun initialize() {
        telemetry.addData("Staus", "FirstKotlinOp Initialized")
        colorSensor = hardwareMap.get(RevColorSensorV3::class.java, "color1")
        dcMotor = hardwareMap.get(DcMotor::class.java, "leftFront")
        imu = hardwareMap.get(IMU::class.java, "imu")
    }

    @OptIn(ExperimentalStdlibApi::class)
    fun runInLoop() {
        telemetry.addData("Status", "Runtime: " + runtime.toString())
        telemetry.addData("Color", "${colorSensor.alpha()}, ${colorSensor.red()}, ${colorSensor.green()}, ${colorSensor.blue()}")
        telemetry.addData("ARGB", "${colorSensor.argb().toHexString()}")
        telemetry.addData("Distance", "${colorSensor.getDistance(DistanceUnit.MM)}")
        telemetry.addData("Motor", dcMotor.currentPosition)
        telemetry.addData("IMY", imu.getRobotAngularVelocity(AngleUnit.DEGREES))

        telemetry.update()
    }

    override fun runOpMode() {
        println("In runOpMode 1")
        initialize()
        println("In runOpMode Atfer initialize")
        // Wait for the game to start (driver presses PLAY)
        waitForStart()
        runtime.reset()
        println("In runOpMode Atfer Start")
        while (opModeIsActive()) {
            runInLoop()
        }
    }
}