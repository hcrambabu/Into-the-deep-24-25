package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Servo: Test", group = "Servo")
public class ServoTest extends LinearOpMode {

    double servoPos = 0.0;
    ServoImplEx servo;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(ServoImplEx.class, "servo");
        //servo.setDirection(Servo.Direction.REVERSE);

        //get our analog input from the hardwareMap
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "servo-0");

        servo.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                setServoPos(0.0);
                telemetry.addData("Key", "A");
            } else if (gamepad1.x) {
                setServoPos(0.5);
                telemetry.addData("Key", "X");
            }
            else if (gamepad1.b) {
                setServoPos(1.0);
                telemetry.addData("Key", "B");
            }
            else if (gamepad1.dpad_right) {
                stepUpServoPos();
                telemetry.addData("Key", "RT");
            }
            else if (gamepad1.dpad_left) {
                stepDownServoPos();
                telemetry.addData("Key", "RL");
            } else if (gamepad1.start) {
                servo.setDirection(Servo.Direction.FORWARD);
            } else if (gamepad1.back) {
                servo.setDirection(Servo.Direction.REVERSE);
            }
            // get the voltage of our analog line
            // divide by 3.3 (the max voltage) to get a value between 0 and 1
            // multiply by 360 to convert it to 0 to 360 degrees
            double position = analogInput.getVoltage() / 3.3 * 360;
//            double position = analogInput.getVoltage() / 3.3;

            telemetry.addData("Direction", servo.getDirection().name());
            telemetry.addData("Pos", servo.getPosition());
            telemetry.addData("Act.Pos", position);
            telemetry.update();
        }
    }

    public void setServoPos(double pos) {
        this.servoPos = pos;
        this.servo.setPosition(this.servoPos);
    }

    public void stepDownServoPos() {
        if(timer.milliseconds() < 100)
            return;
        this.servoPos -= 0.01;
        if (this.servoPos < 0.0) {
            this.servoPos = 0.0;
        }
        this.servo.setPosition(this.servoPos);
        timer.reset();
    }

    public void stepUpServoPos() {
        if(timer.milliseconds() < 100)
            return;
        this.servoPos += 0.01;
        if (this.servoPos > 1.0) {
            this.servoPos = 1.0;
        }
        this.servo.setPosition(this.servoPos);
        timer.reset();
    }
}
