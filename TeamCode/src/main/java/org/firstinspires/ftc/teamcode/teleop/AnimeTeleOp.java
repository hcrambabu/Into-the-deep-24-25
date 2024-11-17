package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_H_H_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_H_V_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_0;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_1;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_2;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_V_BACK_POS;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.anime.AnimeRobot;

@TeleOp(group = "Anime", name = "Anime: TeleOp")
public class AnimeTeleOp extends LinearOpMode {

    public static final double SLOW_RUN_MULTIPLIER = 0.2;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor liftLeft, liftRight, slideLeft, slideRight;
    CRServo intakeContinuousServo;
    Servo intakeLiftServo, dropServo, intakeVerticalTurnServo, intakeHorizontalTurnServo, intakeClawServo;

    IMU imu;
    RevColorSensorV3 intakeColorSensor;
    HuskyLens intakeHuskuy;

    double intakeLiftServoPos = INTAKE_LIFT_DOWN_POS_0;
    double intakeVerticalTurnServoPos = INTAKE_V_BACK_POS;
    double intakeHorizontalTurnServoPos = INTAKE_H_H_POS;
    double intakeClawServoPos = 0.0;

    private ElapsedTime intakeDpadDownRuntime = new ElapsedTime();

    AnimeRobot robot;

    private void initialize() {
        this.robot = new AnimeRobot(this, hardwareMap);

        this.frontLeftMotor = this.robot.getFrontLeftMotor();
        this.backLeftMotor = this.robot.getBackLeftMotor();
        this.frontRightMotor = this.robot.getFrontRightMotor();
        this.backRightMotor = this.robot.getBackRightMotor();

        this.liftLeft = this.robot.getLiftLeft();
        this.liftRight = this.robot.getLiftRight();
        this.slideLeft = this.robot.getSlideLeft();
        this.slideRight = this.robot.getSlideRight();

        this.intakeContinuousServo = this.robot.getIntakeContinousServo();
        this.intakeLiftServo = this.robot.getIntakeLiftServo();
        this.dropServo = this.robot.getDropServo();

        this.imu = this.robot.getImu();
        this.intakeColorSensor = this.robot.getIntakeColorSensor();
        this.intakeHuskuy = this.robot.getIntakeHuskuy();

        this.intakeVerticalTurnServo = this.robot.getIntakeVerticalTurnServo();
        this.intakeHorizontalTurnServo = this.robot.getIntakeHorizontalTurnServo();
        this.intakeClawServo = this.robot.getIntakeClawServo();

        this.intakeVerticalTurnServo.setPosition(INTAKE_V_BACK_POS);
        this.intakeHorizontalTurnServo.setPosition(INTAKE_H_H_POS);
        this.intakeLiftServo.setPosition(INTAKE_LIFT_DOWN_POS_0);
    }

    private void addTelemetry() {
        telemetry.addData("Lift",
                String.format("left: %7d, right:%7d", liftLeft.getCurrentPosition(), liftRight.getCurrentPosition()));
        telemetry.addData("Slide",
                String.format("left: %7d, right:%7d", slideLeft.getCurrentPosition(), slideRight.getCurrentPosition()));
//        telemetry.addData("IntakeColor", String.format("R:%02X , G:%02X, B:%02X",
//                intakeColorSensor.red(), intakeColorSensor.green(), intakeColorSensor.blue()));
//        telemetry.addData("IntakeDistance", intakeColorSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("IntakeLiftServo", intakeLiftServo.getPosition());
        telemetry.addData("IntakeV", intakeVerticalTurnServo.getPosition());
        telemetry.addData("IntakeH", intakeHorizontalTurnServo.getPosition());
        telemetry.addData("Claw", intakeClawServo.getPosition());
        telemetry.addData("DropServo", dropServo.getPosition());
        telemetry.addData("intakeDpadDownRuntime", intakeDpadDownRuntime.seconds());
    }

    private void runLoop() {
        handleMecanum();
        handleDrop();
        handleIntake();
        handleLift();
        handleSlide();
        handleStartButton();
        handleBackButton();
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

        if (gamepad1.left_stick_button) {
            frontLeftPower *= SLOW_RUN_MULTIPLIER;
            backLeftPower *= SLOW_RUN_MULTIPLIER;
            frontRightPower *= SLOW_RUN_MULTIPLIER;
            backRightPower *= SLOW_RUN_MULTIPLIER;
        }

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

    private void handleStartButton() {
        if(gamepad2.start) {
            this.intakeClawServoPos = 1.0;
            this.intakeHorizontalTurnServoPos = INTAKE_H_H_POS;
            this.intakeVerticalTurnServoPos = 1.0;
            this.intakeLiftServoPos = INTAKE_LIFT_DOWN_POS_1;
            intakeDpadDownRuntime.reset();
        }
    }

    private void handleBackButton() {
        if(gamepad2.back) {
            this.intakeClawServoPos = 0.0;
            this.intakeHorizontalTurnServoPos = INTAKE_H_H_POS;
            this.intakeVerticalTurnServoPos = INTAKE_V_BACK_POS;
            this.intakeLiftServoPos = INTAKE_LIFT_DOWN_POS_0;

            this.robot.getIntake().goBack();
        }
    }

    private void handleIntake() {
//        double intakeServoPower = gamepad2.right_trigger - gamepad2.left_trigger;
//        intakeContinuousServo.setPower(intakeServoPower);

        // Intake Vertical Turn Servo
        if(gamepad2.dpad_right) {
            this.intakeVerticalTurnServoPos = INTAKE_V_BACK_POS;
        } else if (gamepad2.dpad_left) {
            this.intakeVerticalTurnServoPos = 1.0;
        }

        // Intake Lift Servo
        if (gamepad2.dpad_down) {
            if(this.intakeLiftServo.getPosition() == INTAKE_LIFT_DOWN_POS_1 && intakeDpadDownRuntime.seconds() > 1.0) {
                this.intakeLiftServoPos = INTAKE_LIFT_DOWN_POS_2;
            } else if(this.intakeLiftServo.getPosition() == INTAKE_LIFT_DOWN_POS_0) {
                this.intakeLiftServoPos = INTAKE_LIFT_DOWN_POS_1;
                intakeDpadDownRuntime.reset();
            }
        } else if (gamepad2.dpad_up) {
            this.intakeLiftServoPos = INTAKE_LIFT_DOWN_POS_0;
        }

        // Intake Horizontal Turn Servo
        if(gamepad2.left_trigger > 0.5) {
            this.intakeHorizontalTurnServoPos -= 0.01;
        } else if (gamepad2.right_trigger > 0.5) {
            this.intakeHorizontalTurnServoPos += 0.01;
        }
        if(this.intakeHorizontalTurnServoPos > INTAKE_H_V_POS) {
            this.intakeHorizontalTurnServoPos = INTAKE_H_V_POS;
        } else if (this.intakeHorizontalTurnServoPos < 0.0) {
            this.intakeHorizontalTurnServoPos = 0.0;
        }

        // Claw Servo
        if(gamepad2.right_bumper) {
            this.intakeClawServoPos = 0.0;
        } else if (gamepad2.left_bumper) {
            this.intakeClawServoPos = 1.0;
        }

        this.intakeClawServo.setPosition(this.intakeClawServoPos);
        this.intakeVerticalTurnServo.setPosition(this.intakeVerticalTurnServoPos);
        this.intakeLiftServo.setPosition(this.intakeLiftServoPos);
        this.intakeHorizontalTurnServo.setPosition(this.intakeHorizontalTurnServoPos);
    }

    private void handleLift() {
        double liftPower = -gamepad2.left_stick_y;
        liftPower = this.robot.getLift().checkLiftPower(liftPower);

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
