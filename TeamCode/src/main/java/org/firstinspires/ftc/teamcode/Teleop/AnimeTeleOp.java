package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.anime.AnimeRobot;

import java.security.acl.Group;

@TeleOp(group = "Anime", name = "Anime: TeleOp")
public class AnimeTeleOp extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor liftLeft, liftRight, slideLeft, slideRight;
    CRServo intakeServo;
    Servo intakeLiftServo, dropServo;

    IMU imu;
    RevColorSensorV3 intakeColorSensor;
    HuskyLens intakeHuskuy;

    double intakeLiftServoPos = 0.0;

    AnimeRobot robot;

    private void initialize() {
        this.robot = new AnimeRobot(hardwareMap);

        this.frontLeftMotor = this.robot.getFrontLeftMotor();
        this.backLeftMotor = this.robot.getBackLeftMotor();
        this.frontRightMotor = this.robot.getFrontRightMotor();
        this.backRightMotor = this.robot.getBackRightMotor();

        this.liftLeft = this.robot.getLiftLeft();
        this.liftRight = this.robot.getLiftRight();
        this.slideLeft = this.robot.getSlideLeft();
        this.slideRight = this.robot.getSlideRight();

        this.intakeServo = this.robot.getIntakeServo();
        this.intakeLiftServo = this.robot.getIntakeLiftServo();
        this.dropServo = this.robot.getDropServo();

        this.imu = this.robot.getImu();
        this.intakeColorSensor = this.robot.getIntakeColorSensor();
        this.intakeHuskuy = this.robot.getIntakeHuskuy();
    }

    private void addTelemetry() {
        telemetry.addData("Lift",
                String.format("left: %7d, right:%7d", liftLeft.getCurrentPosition(), liftRight.getCurrentPosition()));
        telemetry.addData("Slide",
                String.format("left: %7d, right:%7d", slideLeft.getCurrentPosition(), slideRight.getCurrentPosition()));
        telemetry.addData("IntakeColor", String.format("R:%02X , G:%02X, B:%02X",
                intakeColorSensor.red(), intakeColorSensor.green(), intakeColorSensor.blue()));
        telemetry.addData("IntakeDistance", intakeColorSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("IntakeLeftServo", this.intakeLiftServoPos +"-"+intakeLiftServo.getPosition());
        telemetry.addData("DropServo", dropServo.getPosition());
        telemetry.addData("gamepad1 dpad", gamepad1.dpad_up + "" + gamepad1.dpad_down);
        telemetry.addData("gamepad2 dpad", gamepad2.dpad_up + "" + gamepad2.dpad_down);
    }

    private void runLoop() {
        handleMecanum();
        handleDrop();
        handleIntake();
        handleLift();
        handleSlide();
        addTelemetry();
    }

    private void handleMecanum() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void handleSlide() {
        double slidePower = -gamepad2.right_stick_y;
        slidePower = this.robot.getIntake().checkSlidePower(slidePower);

        slideLeft.setPower(slidePower);
        slideRight.setPower(slidePower);
    }

    private void handleIntake() {
        double intakeServoPower = gamepad2.right_trigger - gamepad2.left_trigger;
        intakeServo.setPower(intakeServoPower);

        System.out.println(gamepad1.dpad_up + "" + gamepad1.dpad_down);
        if(gamepad2.dpad_up) {
            this.intakeLiftServoPos -= 0.02;
        } else if (gamepad2.dpad_down) {
            this.intakeLiftServoPos += 0.02;
        }

        if (gamepad2.dpad_left) {
            this.intakeLiftServoPos = 1.0;
        } else if (gamepad2.dpad_right) {
            this.intakeLiftServoPos = 0.0;
        }
        if(this.intakeLiftServoPos < 0.0)
            this.intakeLiftServoPos = 0.0;
        else if(this.intakeLiftServoPos > 1.0) {
            this.intakeLiftServoPos = 1.0;
        }
        this.intakeLiftServo.setPosition(this.intakeLiftServoPos);
    }

    private void handleLift() {
        double liftPower = -gamepad2.left_stick_y;
//        if(liftPower > 0 && (liftLeft.getCurrentPosition() > 4000 || liftRight.getCurrentPosition() > 4000)) {
//            liftPower = 0.1;
//        } else if (liftPower < 0 && (liftLeft.getCurrentPosition() < 200 || liftRight.getCurrentPosition() < 200)) {
//            liftPower = -0.1;
//        }
        liftLeft.setPower(liftPower);
        liftRight.setPower(liftPower);
    }

    private void handleDrop() {
        System.out.println(gamepad1.dpad_up + "" + gamepad1.dpad_down);
        if(gamepad1.dpad_up) {
            this.dropServo.setPosition(0.0);
        } else if (gamepad1.dpad_down) {
            this.dropServo.setPosition(1.0);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        this.initialize();
        waitForStart();

        while (opModeIsActive()) {
            runLoop();
            telemetry.update();
        }
    }
}
