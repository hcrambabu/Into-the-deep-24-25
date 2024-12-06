package org.firstinspires.ftc.teamcode.anime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

import java.util.logging.Logger;


public class AnimeRobot {

    public static final int LIFT_MAX_HEIGHT = 4200;
    public static final int LIFT_MIN_HEIGHT = 100;
    public static final int LIFT_SPECIMEN_PICK_UP_HEIGHT = 0;
    public static final int LIFT_SPECIMEN_HANG_HEIGHT = 1500;
    public static final int SLIDE_MAX_LENGTH = 1900;
    public static final int SLIDE_MIN_LENGTH = 100;
    public static final double MIN_SLIDE_POWER = 0.04;
    public static final double MIN_LIFT_POWER = 0.07;
    public static final double INTAKE_V_BACK_POS = 0.35;
    public static final double INTAKE_H_H_POS = 0.24;
    public static final double INTAKE_H_V_POS = 0.62;
    public static final double INTAKE_LIFT_DOWN_POS_0 = 0.0; //0.68; GB value
    public static final double INTAKE_LIFT_DOWN_POS_1 = 0.45; //0.92; GB value
    public static final double INTAKE_LIFT_DOWN_POS_2 = 0.55; //1.0; GB value
    public static final double DROP_SERVO_MIN_POS = 0.0;
    public static final double DROP_SERVO_MAX_POS = 1.0;
    public static final double DROP_SERVO_INIT_POS = 0.08;
    public static final double DROP_SERVO_OUT_POS = 0.85;
    public static final double DROP_SERVO_SPECIMEN_COLLECT_POS = 0.76;
    public static final double DROP_SERVO_SPECIMEN_HANG_POS = DROP_SERVO_OUT_POS;
    public static final double DROP_CLAW_MAX_POS = 1.0;
    public static final double DROP_CLAW_MIN_POS = 0.0;
    public static final double DROP_CLAW_CLOSE_POS = DROP_CLAW_MIN_POS;
    public static final double DROP_CLAW_OPEN_POS = DROP_CLAW_MAX_POS;
    public static final double INTAKE_CLAW_CLOSE_POS = 0.0;
    public static final double INTAKE_CLAW_OPEN_POS = 1.0;
    public static final double RESET_WAIT_TIME = 2.0;
    public static final double MOTORS_ZERO_VELOCITY = 10;
    public static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
    public static final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private static Logger log = Logger.getLogger(AnimeRobot.class.getName());
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotorEx liftLeft, liftRight, slideLeft, slideRight;
    CRServo intakeContinousServo;
    Servo intakeLiftServo, dropServo, dropClaw, intakeVerticalTurnServo, intakeHorizontalTurnServo, intakeClawServo;
    IMU imu;
    RevColorSensorV3 intakeColorSensor;
    HuskyLens intakeHuskuy;
    GoBildaPinpointDriverRR pinpoint;
    HardwareMap hardwareMap;

    boolean holdLift = false;
    boolean hangTheBot = false;
    Intake intake;
    Lift lift;
    PinpointDrive drive;
    double intakeLiftServoPos = INTAKE_LIFT_DOWN_POS_0;
    double intakeVerticalTurnServoPos = INTAKE_V_BACK_POS;
    double intakeHorizontalTurnServoPos = INTAKE_H_H_POS;
    double intakeClawServoPos = INTAKE_CLAW_CLOSE_POS;
    double dropClawServoPos = DROP_CLAW_OPEN_POS;
    double dropServoPos = DROP_SERVO_INIT_POS;
    private LinearOpMode opMode;
    private ElapsedTime robotRuntime = new ElapsedTime();
    public AnimeRobot(LinearOpMode opMode, Pose2d beginPose) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        this.backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeContinousServo = hardwareMap.crservo.get("intakeContinuous");
        intakeLiftServo = hardwareMap.servo.get("intakeLift");
        dropServo = hardwareMap.servo.get("dropServo");
        dropClaw = hardwareMap.servo.get("dropClaw");
        //intakeLiftServo.setDirection(Servo.Direction.REVERSE); --> For GB Reverse

        imu = hardwareMap.get(IMU.class, "imu");
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        intakeHuskuy = hardwareMap.get(HuskyLens.class, "intakeHusky");
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");

        intakeVerticalTurnServo = hardwareMap.servo.get("intakeV");
        intakeHorizontalTurnServo = hardwareMap.servo.get("intakeH");
        intakeClawServo = hardwareMap.servo.get("intakeClaw");

        this.intakeVerticalTurnServo.setPosition(intakeVerticalTurnServoPos);
        this.intakeHorizontalTurnServo.setPosition(intakeHorizontalTurnServoPos);
        this.intakeLiftServo.setPosition(intakeLiftServoPos);
        this.dropServo.setPosition(dropServoPos);
        this.dropClaw.setPosition(dropClawServoPos);

        this.drive = new PinpointDrive(hardwareMap, beginPose);
        this.intake = new Intake(this);
        this.lift = new Lift(this);

        log.info("Before resetSlideNLift....");
        this.resetSlideNLift();
        log.info("After resetSlideNLift....");
    }

    public boolean isHangTheBot() {
        return hangTheBot;
    }

    public void setHangTheBot(boolean hangTheBot) {
        this.hangTheBot = hangTheBot;
    }

    public boolean isHoldLift() {
        return holdLift;
    }

    public void setHoldLift(boolean holdLift) {
        this.holdLift = holdLift;
    }

    public GoBildaPinpointDriverRR getPinpoint() {
        return pinpoint;
    }

    public void resetEncoders() {
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public DcMotorEx getFrontLeftMotor() {
        return frontLeftMotor;
    }

    public DcMotorEx getBackLeftMotor() {
        return backLeftMotor;
    }

    public DcMotorEx getFrontRightMotor() {
        return frontRightMotor;
    }

    public DcMotorEx getBackRightMotor() {
        return backRightMotor;
    }

    public DcMotorEx getLiftLeft() {
        return liftLeft;
    }

    public DcMotorEx getLiftRight() {
        return liftRight;
    }

    public DcMotorEx getSlideLeft() {
        return slideLeft;
    }

    public DcMotorEx getSlideRight() {
        return slideRight;
    }

    public CRServo getIntakeContinousServo() {
        return intakeContinousServo;
    }

    public Servo getIntakeLiftServo() {
        return intakeLiftServo;
    }

    public Servo getDropServo() {
        return dropServo;
    }

    public IMU getImu() {
        return imu;
    }

    public RevColorSensorV3 getIntakeColorSensor() {
        return intakeColorSensor;
    }

    public HuskyLens getIntakeHuskuy() {
        return intakeHuskuy;
    }

    public Intake getIntake() {
        return intake;
    }

    public Lift getLift() {
        return lift;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public Servo getIntakeVerticalTurnServo() {
        return intakeVerticalTurnServo;
    }

    public Servo getDropClaw() {
        return dropClaw;
    }

    public ElapsedTime getRobotRuntime() {
        return robotRuntime;
    }

    public Servo getIntakeHorizontalTurnServo() {
        return intakeHorizontalTurnServo;
    }

    public Servo getIntakeClawServo() {
        return intakeClawServo;
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public double getIntakeVerticalTurnServoPos() {
        return intakeVerticalTurnServoPos;
    }

    public void setIntakeVerticalTurnServoPos(double intakeVerticalTurnServoPos) {
        this.intakeVerticalTurnServoPos = intakeVerticalTurnServoPos;
    }

    public double getIntakeHorizontalTurnServoPos() {
        return intakeHorizontalTurnServoPos;
    }

    public void setIntakeHorizontalTurnServoPos(double intakeHorizontalTurnServoPos) {
        this.intakeHorizontalTurnServoPos = intakeHorizontalTurnServoPos;
    }

    public double getIntakeClawServoPos() {
        return intakeClawServoPos;
    }

    public void setIntakeClawServoPos(double intakeClawServoPos) {
        this.intakeClawServoPos = intakeClawServoPos;
    }

    public double getDropClawServoPos() {
        return dropClawServoPos;
    }

    public void setDropClawServoPos(double dropClawServoPos) {
        this.dropClawServoPos = dropClawServoPos;
    }

    public double getDropServoPos() {
        return dropServoPos;
    }

    public void setDropServoPos(double dropServoPos) {
        this.dropServoPos = dropServoPos;
    }

    public double getIntakeLiftServoPos() {
        return intakeLiftServoPos;
    }

    public void setIntakeLiftServoPos(double intakeLiftServoPos) {
        this.intakeLiftServoPos = intakeLiftServoPos;
    }

    public double stepDownIntakeHorizantalServoPos() {
        this.intakeHorizontalTurnServoPos -= 0.01;
        if (this.intakeHorizontalTurnServoPos < 0.0) {
            this.intakeHorizontalTurnServoPos = 0.0;
        }
        return this.intakeHorizontalTurnServoPos;
    }

    public double stepUpIntakeHorizantalServoPos() {
        this.intakeHorizontalTurnServoPos += 0.01;
        if (this.intakeHorizontalTurnServoPos > INTAKE_H_V_POS) {
            this.intakeHorizontalTurnServoPos = INTAKE_H_V_POS;
        }

        return this.intakeHorizontalTurnServoPos;
    }

    public double stepDownDropServoPos() {
        this.dropServoPos -= 0.01;
        if (this.dropServoPos < DROP_SERVO_MIN_POS) {
            this.dropServoPos = DROP_SERVO_MIN_POS;
        }
        return this.dropServoPos;
    }

    public double stepUpDropServoPos() {
        this.dropServoPos += 0.01;
        if (this.dropServoPos > DROP_SERVO_MAX_POS) {
            this.dropServoPos = DROP_SERVO_MAX_POS;
        }
        return this.dropServoPos;
    }

    public PinpointDrive getDrive() {
        return drive;
    }

    public void resetSlideNLift() {
        this.lift.resetLift();
        this.intake.resetIntake();
    }

    public void resetMotors(String name, DcMotorEx left, DcMotorEx right, ElapsedTime timer) {
        log.info(String.format("In resetMotors {%s} ...", name));
        left.setPower(-1);
        right.setPower(-1);
        try {
            Thread.sleep(100);
        } catch (InterruptedException ex) {
        } // Wait little time for motors to start
        timer.reset();
        while (timer.seconds() < RESET_WAIT_TIME &&
                Math.abs(left.getVelocity()) > MOTORS_ZERO_VELOCITY && Math.abs(right.getVelocity()) > MOTORS_ZERO_VELOCITY) {
            Thread.yield();
            log.info(String.format("In resetMotors {%s} ... Waiting for zero velocity ...", name));
        }
        left.setPower(0);
        right.setPower(0);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        log.info(String.format("Out resetMotors {%s} ...", name));
    }
}
