package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_INIT_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_OUT_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_0;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_1;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_2;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_V_BACK_POS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;

import java.util.logging.Logger;

@TeleOp(group = "Anime", name = "Anime: TeleOp")
public class AnimeTeleOp extends BaseOpMode {

    public static final double SLOW_RUN_MULTIPLIER = 0.2;
    private static Logger log = Logger.getLogger(AnimeTeleOp.class.getName());
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotorEx liftLeft, liftRight, slideLeft, slideRight;
    CRServo intakeContinuousServo;
    Servo intakeLiftServo, dropServo, dropClaw, intakeVerticalTurnServo, intakeHorizontalTurnServo, intakeClawServo;

    IMU imu;
    RevColorSensorV3 intakeColorSensor;
    HuskyLens intakeHuskuy;

    private ElapsedTime intakeDpadDownRuntime = new ElapsedTime();
    private long logCount = 0;
    private long maxlogCount = 10;

    @Override
    public void initialize(Pose2d beginPose) {
        super.initialize(beginPose);

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
        this.dropClaw = this.robot.getDropClaw();

        this.imu = this.robot.getImu();
        this.intakeColorSensor = this.robot.getIntakeColorSensor();
        this.intakeHuskuy = this.robot.getIntakeHuskuy();

        this.intakeVerticalTurnServo = this.robot.getIntakeVerticalTurnServo();
        this.intakeHorizontalTurnServo = this.robot.getIntakeHorizontalTurnServo();
        this.intakeClawServo = this.robot.getIntakeClawServo();

        this.intakeVerticalTurnServo.setPosition(this.robot.getIntakeVerticalTurnServoPos());
        this.intakeHorizontalTurnServo.setPosition(this.robot.getIntakeHorizontalTurnServoPos());
        this.intakeLiftServo.setPosition(this.robot.getIntakeLiftServoPos());
        this.dropServo.setPosition(this.robot.getDropServoPos());
        this.dropClaw.setPosition(this.robot.getDropClawServoPos());
    }

    private void addTelemetry() {
        if (!this.robot.isHangTheBot()) {
            telemetry.addLine();
            telemetry.addData("LiftPos",
                    String.format("left: %7d, right:%7d", liftLeft.getCurrentPosition(), liftRight.getCurrentPosition()));
            telemetry.addData("LiftVel",
                    String.format("left: %.3f, right:%.3f", liftLeft.getVelocity(), liftRight.getVelocity()));
            telemetry.addLine();
            telemetry.addData("SlidePos",
                    String.format("left: %7d, right:%7d", slideLeft.getCurrentPosition(), slideRight.getCurrentPosition()));
            telemetry.addData("SlideVel",
                    String.format("left: %.3f, right:%.3f", slideLeft.getVelocity(), slideRight.getVelocity()));
            telemetry.addLine();
//        telemetry.addData("IntakeColor", String.format("R:%02X , G:%02X, B:%02X",
//                intakeColorSensor.red(), intakeColorSensor.green(), intakeColorSensor.blue()));
//        telemetry.addData("IntakeDistance", intakeColorSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("IntakeLiftServo", intakeLiftServo.getPosition());
            telemetry.addData("IntakeV", intakeVerticalTurnServo.getPosition());
            telemetry.addData("IntakeH", intakeHorizontalTurnServo.getPosition());
            telemetry.addLine();
            telemetry.addData("intakeDpadDownRuntime", intakeDpadDownRuntime.seconds());
            telemetry.addLine();
            telemetry.addData("Claw", intakeClawServo.getPosition());
            telemetry.addData("DropServo", dropServo.getPosition());
            telemetry.addLine();

        } else {
            telemetry.addLine();
            telemetry.addData("1", "##################################");
            telemetry.addLine("Robot is HANGING");
            telemetry.addData("Power", liftLeft.getPower() + ", " + liftRight.getPower());
            telemetry.addData("Mode", liftLeft.getMode() + ", " + liftRight.getMode());
            telemetry.addData("2", "##################################");
            telemetry.addLine();
        }
    }

    public void runLoop() throws InterruptedException {
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

//        this.robot.getDrive().setDrivePowers(new PoseVelocity2d(
//                new Vector2d(
//                        -gamepad1.left_stick_y,
//                        -gamepad1.left_stick_x
//                ),
//                -gamepad1.right_stick_x
//        ));
//
//        this.robot.getDrive().updatePoseEstimate();
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
        this.robot.getIntake().handleSlideManually(-gamepad2.right_stick_y);
    }

    private void handleStartButton() {
        if (gamepad2.start) {
            intakeDpadDownRuntime.reset();
            this.robot.getIntake().goFront();
        }

        if (gamepad1.start) {
            this.robot.getLift().goToDrop();
        }
    }

    private void handleBackButton() {
        if (gamepad2.back) {
            this.robot.getIntake().goBack();
        }

        if (gamepad1.back) {
            this.robot.getLift().goBack();
        }
    }

    private void handleIntake() {
        // Intake Vertical Turn Servo
        if (gamepad2.dpad_right) {
            this.robot.setIntakeVerticalTurnServoPos(INTAKE_V_BACK_POS);
        } else if (gamepad2.dpad_left) {
            this.robot.setIntakeVerticalTurnServoPos(1.0);
        }

        // Intake Lift Servo
        if (gamepad2.dpad_down) {
            if (this.intakeLiftServo.getPosition() == INTAKE_LIFT_DOWN_POS_1 && intakeDpadDownRuntime.seconds() > 1.0) {
                this.robot.setIntakeLiftServoPos(INTAKE_LIFT_DOWN_POS_2);
            } else if (this.intakeLiftServo.getPosition() == INTAKE_LIFT_DOWN_POS_0) {
                this.robot.setIntakeLiftServoPos(INTAKE_LIFT_DOWN_POS_1);
                intakeDpadDownRuntime.reset();
            }
        } else if (gamepad2.dpad_up) {
            this.robot.setIntakeLiftServoPos(INTAKE_LIFT_DOWN_POS_0);
        }

        // Intake Horizontal Turn Servo
        if (gamepad2.left_trigger > 0.5) {
            this.robot.stepDownIntakeHorizantalServoPos();
        } else if (gamepad2.right_trigger > 0.5) {
            this.robot.stepUpIntakeHorizantalServoPos();
        }

        // Claw Servo
        if (gamepad2.right_bumper) {
            this.robot.setIntakeClawServoPos(INTAKE_CLAW_CLOSE_POS);
        } else if (gamepad2.left_bumper) {
            this.robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
        }

        this.intakeClawServo.setPosition(this.robot.getIntakeClawServoPos());
        this.intakeVerticalTurnServo.setPosition(this.robot.getIntakeVerticalTurnServoPos());
        this.intakeHorizontalTurnServo.setPosition(this.robot.getIntakeHorizontalTurnServoPos());
        this.intakeLiftServo.setPosition(this.robot.getIntakeLiftServoPos());
    }

    private void handleLift() {
        if (gamepad1.a || gamepad2.a) {
//            this.robot.getLiftLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            this.robot.getLiftRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            this.robot.setHangTheBot(true);
            this.robot.getLift().goToDrop();
        } else if (gamepad1.b || gamepad2.b) {
//            this.robot.getLiftLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            this.robot.getLiftRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            this.robot.setHangTheBot(false);
            this.robot.getLift().goBack();
        } else if (gamepad1.x || gamepad2.x) {
            this.robot.getLift().specimenPickup();
        } else if (gamepad1.y || gamepad2.y) {
            this.robot.getLift().specimenDrop();
        }

        if (logCount == maxlogCount) {
            log.info("In handleLift....");
        }
        this.robot.getLift().handleLiftManually(-gamepad2.left_stick_y);
    }

    private void handleDrop() {

        // DropClaw Servo
        if (gamepad1.right_bumper) {
            this.robot.setDropClawServoPos(DROP_CLAW_CLOSE_POS);
        } else if (gamepad1.left_bumper) {
            this.robot.setDropClawServoPos(DROP_CLAW_OPEN_POS);
        }

        // Drop Servo (Hand)
        if (gamepad1.dpad_up) {
            this.robot.stepDownDropServoPos();
        } else if (gamepad1.dpad_down) {
            this.robot.stepUpDropServoPos();
        } else if (gamepad1.dpad_right) {
            this.robot.setDropServoPos(DROP_SERVO_INIT_POS);
        } else if (gamepad1.dpad_left) {
            this.robot.setDropServoPos(DROP_SERVO_OUT_POS);
        }

        this.dropServo.setPosition(this.robot.getDropServoPos());
        this.dropClaw.setPosition(this.robot.getDropClawServoPos());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("Before initialize....");
        this.initialize(new Pose2d(0, 0, 0)); // TODO get from autonomous pose
        log.info("before Start....");
        waitForStart();
        log.info("After Start....");

        while (opModeIsActive()) {
            runLoop();
            telemetry.update();
        }
    }
}
