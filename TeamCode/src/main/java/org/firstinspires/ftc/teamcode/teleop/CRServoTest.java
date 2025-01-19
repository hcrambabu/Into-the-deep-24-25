package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "CRServo: Test", group = "Servo")
public class CRServoTest extends LinearOpMode {
    CRServo servo;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "crservo");
        //servo.setDirection(Servo.Direction.REVERSE);

        //get our analog input from the hardwareMap
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "crservo-1");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo.setPower(0.5);
                telemetry.addData("Key", "A");
            } else if (gamepad1.x) {
                servo.setPower(0.0);
                telemetry.addData("Key", "X");
            }
            else if (gamepad1.b) {
                servo.setPower(-0.5);
                telemetry.addData("Key", "B");
            } else {
                servo.setPower(0.0);
            }

            // get the voltage of our analog line
            // divide by 3.3 (the max voltage) to get a value between 0 and 1
            // multiply by 360 to convert it to 0 to 360 degrees
            double position = analogInput.getVoltage() / 3.3 * 360;
//            double position = analogInput.getVoltage() / 3.3;

            telemetry.addData("Power", servo.getPower());
            telemetry.addData("Direction", servo.getDirection());
            telemetry.addData("Act.Pos", position);
            telemetry.update();
        }
    }
}
