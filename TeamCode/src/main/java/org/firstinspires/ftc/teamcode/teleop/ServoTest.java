package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Servo: Test", group = "Servo")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "servo");
        //servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo.setPosition(0);
                telemetry.addData("Key", "A");
            } else if (gamepad1.x) {
                servo.setPosition(0.5);
                telemetry.addData("Key", "X");
            }
            else if (gamepad1.b) {
                servo.setPosition(1.0);
                telemetry.addData("Key", "B");
            }
            telemetry.addData("Pos", servo.getPosition());
            telemetry.update();
        }
    }
}
