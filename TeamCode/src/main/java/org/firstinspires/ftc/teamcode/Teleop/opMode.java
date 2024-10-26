package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="opMode", group="Linear OpMode")
public class opMode extends LinearOpMode {
    public void runOpMode() {
        DcMotor DcMotorA = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor DcMotorB = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor DcMotorC = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor DcMotorD = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotorC.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorD.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Motor A Pos:", DcMotorA.getCurrentPosition());
            telemetry.addData("Motor B Pos:", DcMotorB.getCurrentPosition());
            telemetry.addData("Motor C Pos:", DcMotorC.getCurrentPosition());
            telemetry.addData("Motor D Pos:", DcMotorD.getCurrentPosition());

            if (gamepad1.left_stick_y != 0){
                DcMotorA.setPower(gamepad1.left_stick_y);
                DcMotorB.setPower(gamepad1.left_stick_y);
                DcMotorC.setPower(gamepad1.left_stick_y);
                DcMotorD.setPower(gamepad1.left_stick_y);
            }

            else if (gamepad1.right_stick_x != 0){
                DcMotorA.setPower(-gamepad1.right_stick_x);
                DcMotorB.setPower(-gamepad1.right_stick_x);
                DcMotorC.setPower(gamepad1.right_stick_x);
                DcMotorD.setPower(gamepad1.right_stick_x);

            }

            else if (gamepad1.left_stick_y == 0) {
                DcMotorA.setPower(0);
                DcMotorB.setPower(0);
                DcMotorC.setPower(0);
                DcMotorD.setPower(0);
            }
           if (gamepad1.dpad_left){
                DcMotorA.setPower(1);
                DcMotorB.setPower(-1);
                DcMotorC.setPower(1);
                DcMotorD.setPower(-1);
            }
           if (gamepad1.dpad_right){
                DcMotorA.setPower(-1);
                DcMotorB.setPower(1);
                DcMotorC.setPower(-1);
                DcMotorD.setPower(1);
            }
           if (gamepad1.a){
               DcMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               DcMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               DcMotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               DcMotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               DcMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               DcMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               DcMotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               DcMotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           }
            telemetry.update();
        }
    }
}
