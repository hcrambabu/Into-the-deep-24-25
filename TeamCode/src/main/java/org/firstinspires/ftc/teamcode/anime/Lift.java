package org.firstinspires.ftc.teamcode.anime;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.CompletableFuture;
import java.util.logging.Logger;

public class Lift {

    public static final double LOWER_LIFT_SERVOS_MIN_POS = 0.0;
    public static final double LOWER_LIFT_SERVOS_MAX_POS = 1.0;
    public static final double LOWER_LIFT_INCREMENT = 0.01;
    public static final double UPPER_LIFT_SERVOS_MIN_POS = 0.0;
    public static final double UPPER_LIFT_SERVOS_MAX_POS = 1.0;
    public static final double UPPER_LIFT_INCREMENT = 0.01;
    private static Logger log = Logger.getLogger(Lift.class.getName());
    private AnimeRobot robot;
    private HardwareMap hardwareMap;
    Telemetry telemetry;
    Servo lowerLeftServo, lowerRightServo, upperLeftServo, upperRightServo;
    AnalogInput llsai, lrsai, ulsai, ursai;

    DcMotorEx lowerLiftMotor, upperLiftMotor;
    RevColorSensorV3 specimenColorSensor;
    private CompletableFuture<Void> currentTask;

    private double lowerLiftServoPos = LOWER_LIFT_SERVOS_MIN_POS;
    private double upperLiftServoPos = UPPER_LIFT_SERVOS_MAX_POS;

    private ElapsedTime liftRuntime = new ElapsedTime();
    public Lift(AnimeRobot robot) {
        this.robot = robot;
        this.robot = robot;
        this.telemetry = this.robot.getOpMode().telemetry;
        this.hardwareMap = this.robot.getHardwareMap();

        this.specimenColorSensor = hardwareMap.get(RevColorSensorV3.class, "specimenColor");

        this.lowerLeftServo = hardwareMap.get(Servo.class, "lls"); // Lower Left Servo
        this.lowerRightServo = hardwareMap.get(Servo.class, "lrs"); // Lower Right Servo
        this.upperLeftServo = hardwareMap.get(Servo.class, "uls"); // Upper Left Servo
        this.upperRightServo = hardwareMap.get(Servo.class, "urs"); // Upper Right Servo

        this.lowerLeftServo.setDirection(Servo.Direction.REVERSE);
        this.upperLeftServo.setDirection(Servo.Direction.REVERSE);

        this.llsai = hardwareMap.get(AnalogInput.class, "llsai"); // Lower Left Servo Analog Input
        this.lrsai = hardwareMap.get(AnalogInput.class, "lrsai"); // Lower Right Servo Analog Input
        this.ulsai = hardwareMap.get(AnalogInput.class, "ulsai"); // Upper Left Servo Analog Input
        this.ursai = hardwareMap.get(AnalogInput.class, "ursai"); // Upper Right Servo Analog Input

        setLowerLiftServoPos(lowerLiftServoPos);
        setUpperLiftServoPos(upperLiftServoPos);

        this.lowerLiftMotor = hardwareMap.get(DcMotorEx.class, "llm");
        this.upperLiftMotor = hardwareMap.get(DcMotorEx.class, "ulm");


        this.lowerLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        this.lowerLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.upperLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lowerLiftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.upperLiftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.lowerLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.upperLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Servo getLowerLeftServo() {
        return lowerLeftServo;
    }

    public Servo getLowerRightServo() {
        return lowerRightServo;
    }

    public Servo getUpperLeftServo() {
        return upperLeftServo;
    }

    public Servo getUpperRightServo() {
        return upperRightServo;
    }

    public AnalogInput getLlsai() {
        return llsai;
    }

    public AnalogInput getLrsai() {
        return lrsai;
    }

    public AnalogInput getUlsai() {
        return ulsai;
    }

    public AnalogInput getUrsai() {
        return ursai;
    }

    public RevColorSensorV3 getSpecimenColorSensor() {
        return specimenColorSensor;
    }

    public double getLowerLiftServoPos() {
        return lowerLiftServoPos;
    }

    public double getUpperLiftServoPos() {
        return upperLiftServoPos;
    }

    public void setLowerLiftServoPos(double pos) {
        this.lowerLiftServoPos = pos;
        this.lowerLeftServo.setPosition(this.lowerLiftServoPos);
        this.lowerRightServo.setPosition(this.lowerLiftServoPos);
    }

    public void setUpperLiftServoPos(double pos) {
        this.upperLiftServoPos = pos;
        this.upperLeftServo.setPosition(this.upperLiftServoPos);
        this.upperRightServo.setPosition(this.upperLiftServoPos);
    }

    public void adjustLowerLift(double multiplier) {
        double pos = this.getLowerLiftServoPos() + (LOWER_LIFT_INCREMENT * multiplier);
        if(pos > LOWER_LIFT_SERVOS_MAX_POS) {
            pos = LOWER_LIFT_SERVOS_MAX_POS;
        } else if(pos < LOWER_LIFT_SERVOS_MIN_POS) {
            pos = LOWER_LIFT_SERVOS_MIN_POS;
        }
        setLowerLiftServoPos(pos);
    }

    public void adjustUpperLift(double multiplier) {
        double pos = this.getUpperLiftServoPos() + (UPPER_LIFT_INCREMENT * multiplier);
        if(pos > UPPER_LIFT_SERVOS_MAX_POS) {
            pos = UPPER_LIFT_SERVOS_MAX_POS;
        } else if(pos < UPPER_LIFT_SERVOS_MIN_POS) {
            pos = UPPER_LIFT_SERVOS_MIN_POS;
        }
        setUpperLiftServoPos(pos);
    }

    private double getActPos(AnalogInput ai) {
        return ai.getVoltage() / 3.3 * 360;
    }

    public double getLlsaiPos() {
        return getActPos(llsai);
    }

    public double getLrsaiPos() {
        return getActPos(lrsai);
    }

    public double getUlsaiPos() {
        return getActPos(ulsai);
    }

    public double getUrsaiPos() {
        return getActPos(ursai);
    }

    public void handleKeyPress(Gamepad gamepad1, Gamepad gamepad2) {
//        adjustLowerLift(-gamepad2.left_stick_y);
//        adjustUpperLift(-gamepad2.right_stick_y);

        if(Math.abs(gamepad2.left_stick_y) > 0.5 && this.lowerLiftMotor.getCurrentPosition() < 400) {
            this.lowerLiftMotor.setPower(-gamepad2.left_stick_y);
        } else {
            this.lowerLiftMotor.setPower(0.4);
        }

        this.upperLiftMotor.setPower(-gamepad2.right_stick_y);
        this.telemetry.addData("lift power", "lower: %.3f, upper: %.3f", this.lowerLiftMotor.getPower(), this.upperLiftMotor.getPower());
        this.telemetry.addData("lift pos", "lower: %d, upper: %d", this.lowerLiftMotor.getCurrentPosition(), this.upperLiftMotor.getCurrentPosition());
        updateTelemetry();
    }

    public void updateTelemetry() {
        this.telemetry.addLine();
        this.telemetry.addData("Lower",
                String.format("InPos: %.3f, LPos: %.3f, RPos: %.3f, LAPos:%.3f,  RAPos:%.3f",
                        this.lowerLiftServoPos,
                        this.lowerLeftServo.getPosition(), this.lowerRightServo.getPosition(),
                        getLlsaiPos(), getLrsaiPos()));
        this.telemetry.addData("Upper",
                String.format("InPos: %.3f, LPos: %.3f, RPos: %.3f, LAPos:%.3f,  RAPos:%.3f",
                        this.upperLiftServoPos,
                        this.upperLeftServo.getPosition(), this.upperRightServo.getPosition(),
                        getUlsaiPos(), getUrsaiPos()));
        this.telemetry.addData("specimenDis", String.format("%.3f cm", this.specimenColorSensor.getDistance(DistanceUnit.CM)));
        this.telemetry.addData("specimenColor", ColorUtility.getColorName(this.specimenColorSensor.getNormalizedColors()).name());
    }
}
