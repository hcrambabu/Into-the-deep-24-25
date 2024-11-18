package org.firstinspires.ftc.teamcode.anime;

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


public class AnimeRobot {

    public static final int LIFT_MAX_HEIGHT = 4200;
    public static final int LIFT_MIN_HEIGHT = 100;
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

    public static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
    public static final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotorEx liftLeft, liftRight, slideLeft, slideRight;
    CRServo intakeContinousServo;
    Servo intakeLiftServo, dropServo, intakeVerticalTurnServo, intakeHorizontalTurnServo, intakeClawServo;
    IMU imu;
    RevColorSensorV3 intakeColorSensor;
    HuskyLens intakeHuskuy;
    GoBildaPinpointDriverRR pinpoint;
    HardwareMap hardwareMap;

    Intake intake;
    Lift lift;

    private LinearOpMode opMode;

    private ElapsedTime robotRuntime = new ElapsedTime();

    public GoBildaPinpointDriverRR getPinpoint() {
        return pinpoint;
    }

    public AnimeRobot(LinearOpMode opMode) {
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

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeContinousServo = hardwareMap.crservo.get("intakeContinuous");
        intakeLiftServo = hardwareMap.servo.get("intakeLift");
        dropServo = hardwareMap.servo.get("dropServo");
//        intakeLiftServo.setDirection(Servo.Direction.REVERSE); --> For GB Reverse
        dropServo.setDirection(Servo.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        intakeHuskuy = hardwareMap.get(HuskyLens.class, "intakeHusky");
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class, "pinpoint");

        intakeVerticalTurnServo = hardwareMap.servo.get("intakeV");
        intakeHorizontalTurnServo = hardwareMap.servo.get("intakeH");
        intakeClawServo = hardwareMap.servo.get("intakeClaw");

        this.intakeVerticalTurnServo.setPosition(INTAKE_V_BACK_POS);
        this.intakeHorizontalTurnServo.setPosition(INTAKE_H_H_POS);
        this.intakeLiftServo.setPosition(INTAKE_LIFT_DOWN_POS_0);

        this.intake = new Intake(this);
        this.lift = new Lift(this);
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

    public Servo getIntakeHorizontalTurnServo() {
        return intakeHorizontalTurnServo;
    }

    public Servo getIntakeClawServo() {
        return intakeClawServo;
    }

    public LinearOpMode getOpMode() {
        return opMode;
    }
}
