package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_INIT_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_OUT_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_FACE_DOWN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_FACE_UP_POS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;
import org.firstinspires.ftc.teamcode.anime.PoseStorage;

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
    Servo testServo;
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

        this.testServo = this.robot.getTestServo();
        this.intakeLiftServo = this.robot.getIntakeLiftServo();
        this.dropServo = this.robot.getDropServo();
        this.dropClaw = this.robot.getDropClaw();

        this.imu = this.robot.getImu();
        this.intakeColorSensor = this.robot.getIntakeColorSensor();
        this.intakeHuskuy = this.robot.getIntakeHuskyLens();

        this.intakeVerticalTurnServo = this.robot.getIntakeFaceUpDownServo();
        this.intakeHorizontalTurnServo = this.robot.getIntakeHorizontalTurnServo();
        this.intakeClawServo = this.robot.getIntakeClawServo();
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
            Pose2d rp = this.robot.getDrive().getPose();
            telemetry.addData("Robot", String.format(String.format("X:%.2f, Y:%.2f, A:%.2f", rp.position.x, rp.position.y, Math.toDegrees(rp.heading.toDouble()))));
            telemetry.addLine();
            Pose2d bp = this.robot.getIntakeHusky().getBlockPoseWrtRobot();
            if(bp != null)
                telemetry.addData("Block", String.format(String.format("X:%.2f, Y:%.2f, A:%.2f", bp.position.x, bp.position.y, Math.toDegrees(bp.heading.toDouble()))));
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
//        searchForSample();
        addTelemetry();
    }

    private void searchForSample() {
        if(gamepad1.right_trigger > 0.5) {
            this.robot.getIntake().searchForSample(10, -1);
        }
    }

    private void handleMecanum() {

        double g1ly = -gamepad1.left_stick_y;
        double g1lx = -gamepad1.left_stick_x;
        double g1rx = -gamepad1.right_stick_x;
        if (gamepad1.left_stick_button) {
            g1ly *= SLOW_RUN_MULTIPLIER;
            g1lx *= SLOW_RUN_MULTIPLIER;
            g1rx *= SLOW_RUN_MULTIPLIER;
        }

        this.robot.getDrive().setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        g1ly,
                        g1lx
                ),
                g1rx
        ));

        this.robot.getDrive().updatePoseEstimate();
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.fieldOverlay().setStroke("#3F51B5");
//        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void handleSlide() {
        this.robot.getIntake().handleSlideManually(-gamepad2.right_stick_y);
    }

    private void handleStartButton() {
        if (gamepad2.start) {
            intakeDpadDownRuntime.reset();
            this.robot.getIntake().goFront(true);
        }

        if (gamepad1.start) {
            this.robot.getLift().liftUpToBasketLevel(false);
        }
    }

    private void handleBackButton() {
        if (gamepad2.back) {
            this.robot.getIntake().goBack();
        }

        if (gamepad1.back) {
            this.robot.getLift().liftDown();
        }
    }

    private void handleIntake() {
        // Intake Vertical Turn Servo
        if (gamepad2.dpad_right) {
            this.robot.setIntakeFaceUpDownServoPos(INTAKE_FACE_UP_POS);
        } else if (gamepad2.dpad_left) {
            this.robot.setIntakeFaceUpDownServoPos(INTAKE_FACE_DOWN_POS);
        }

        // Intake Lift Servo
        if (gamepad2.dpad_down) {
            this.robot.stepUpIntakeLiftServoPos();
        } else if (gamepad2.dpad_up) {
            this.robot.stepDownIntakeLiftServoPos();
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
    }

    private void handleLift() {
        if (gamepad1.a || gamepad2.a) {
            this.robot.getLift().liftUpToBasketLevel(false);
        } else if (gamepad1.b || gamepad2.b) {
            this.robot.getLift().liftDown();
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
    }

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("Before initialize....");
        this.initialize(PoseStorage.currentPose); // TODO get from autonomous pose
        log.info("before Start....");
        telemetry.update();
        waitForStart();
        log.info("After Start....");

        while (opModeIsActive()) {
            runLoop();
            telemetry.update();
        }
    }
}
