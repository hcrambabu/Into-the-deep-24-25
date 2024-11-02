package org.firstinspires.ftc.teamcode.anime;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


public class AnimeRobot {

    public static final int LIFT_MAX_HEIGHT = 4000;
    public static final int LIFT_MIN_HEIGHT = 200;

    public static final int SLIDE_MAX_LENGTH = 1900;
    public static final int SLIDE_MIN_LENGTH = 100;
    public static final double MIN_SLIDE_POWER = 0.1;

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
    HardwareMap hardwareMap;

    Intake intake;
    Lift lift;

    public AnimeRobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        this.backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        this.frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        this.backRightMotor = hardwareMap.dcMotor.get("rightBack");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight");
        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");

        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeServo = hardwareMap.crservo.get("intakeServo");
        intakeLiftServo = hardwareMap.servo.get("intakeLift");
        dropServo = hardwareMap.servo.get("dropServo");
        intakeLiftServo.setDirection(Servo.Direction.REVERSE);
        dropServo.setDirection(Servo.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        intakeHuskuy = hardwareMap.get(HuskyLens.class, "intakeHusky");

        this.intake = new Intake(this);
        this.lift = new Lift(this);
    }

    public void resetEncoders() {
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public DcMotor getFrontLeftMotor() {
        return frontLeftMotor;
    }

    public DcMotor getBackLeftMotor() {
        return backLeftMotor;
    }

    public DcMotor getFrontRightMotor() {
        return frontRightMotor;
    }

    public DcMotor getBackRightMotor() {
        return backRightMotor;
    }

    public DcMotor getLiftLeft() {
        return liftLeft;
    }

    public DcMotor getLiftRight() {
        return liftRight;
    }

    public DcMotor getSlideLeft() {
        return slideLeft;
    }

    public DcMotor getSlideRight() {
        return slideRight;
    }

    public CRServo getIntakeServo() {
        return intakeServo;
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
}
