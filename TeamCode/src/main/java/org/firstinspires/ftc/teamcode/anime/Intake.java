package org.firstinspires.ftc.teamcode.anime;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.CompletableFuture;

public class Intake {

    public static final double INTAKE_SERVO_MIN_POS = 0.0;
    public static final double INTAKE_SERVO_MAX_POS = 1.0;
    public static final double INTAKE_SERVO_START_POS = INTAKE_SERVO_MIN_POS;
    public static final double INTAKE_SERVO_INCREMENT = 0.01;

    public static final double CRSERVO_SAMPLE_INTAKE_POWER = 1.0;
    public static final double CRSERVO_SAMPLE_RELEASE_POWER = -1.0;
    public static final double CRSERVO_STOP_POWER = 0.0;

    private AnimeRobot robot;
    private HardwareMap hardwareMap;
    Servo intakeLiftServo;
    CRServo intakeCRServo;
    RevColorSensorV3 intakeColorSensor;

    Telemetry telemetry;

    AnalogInput intakeServoAnalogInput, crServoAnalogInput;
    private CompletableFuture<Void> currentTask;
    private CompletableFuture<Void> releaseSampleTask;
    private double intakeServoPos = INTAKE_SERVO_START_POS;

    public Intake(AnimeRobot robot) {
        this.robot = robot;
        this.hardwareMap = this.robot.getHardwareMap();
        this.intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        this.intakeLiftServo = hardwareMap.get(Servo.class, "ils"); // Intake Lift Servo
        this.intakeCRServo = hardwareMap.get(CRServo.class, "crservo");
        this.intakeServoAnalogInput = hardwareMap.get(AnalogInput.class, "ilsai"); // Intake Lift Servo Actual Position
        this.crServoAnalogInput = hardwareMap.get(AnalogInput.class, "crsai"); // Continuous Servo Actual Position
        this.telemetry = this.robot.getOpMode().telemetry;

        this.intakeLiftServo.setPosition(intakeServoPos);
        this.intakeCRServo.setPower(CRSERVO_STOP_POWER);
    }

    public Servo getIntakeLiftServo() {
        return intakeLiftServo;
    }

    public CRServo getIntakeCRServo() {
        return intakeCRServo;
    }

    public RevColorSensorV3 getIntakeColorSensor() {
        return intakeColorSensor;
    }

    public double getIntakeServoPos() {
        return this.intakeServoPos;
    }

    public void setIntakeServoPos(double intakeServoPos) {
        this.intakeServoPos = intakeServoPos;
        this.intakeLiftServo.setPosition(intakeServoPos);
    }

    public void incrementIntakeServoPos() {
       double pos = this.getIntakeServoPos() + INTAKE_SERVO_INCREMENT;
        if(pos > INTAKE_SERVO_MAX_POS) {
            pos = INTAKE_SERVO_MAX_POS;
        }
        setIntakeServoPos(pos);
    }
    public void decrementIntakeServoPos() {
        double pos = this.getIntakeServoPos() - INTAKE_SERVO_INCREMENT;
        if(pos < INTAKE_SERVO_MIN_POS) {
            pos = INTAKE_SERVO_MIN_POS;
        }
        setIntakeServoPos(pos);
    }

    public void startIntakeCRServo() {
        this.intakeCRServo.setPower(CRSERVO_SAMPLE_INTAKE_POWER);
    }

    public void stopIntakeCRServo() {
        this.intakeCRServo.setPower(CRSERVO_STOP_POWER);
    }

    public void releaseCRServo() {
        this.intakeCRServo.setPower(CRSERVO_SAMPLE_RELEASE_POWER);
    }

    public double getIntakeActPos() {
        return this.intakeServoAnalogInput.getVoltage() / 3.3 * 360;
    }

    public Runnable releaseSample() {
        return () -> {
            releaseCRServo();
            this.robot.sleep(500);
            stopIntakeCRServo();
        };
    }

    public void releaseSampleAsync() {
        releaseSampleTask = AsyncUtility.createAsyncTask(releaseSampleTask, releaseSample());
    }

    public void handleKeyPress(Gamepad gamepad1, Gamepad gamepad2) {
        if(gamepad2.right_trigger > 0.5) {
            incrementIntakeServoPos();
        } else if(gamepad2.left_trigger > 0.5) {
            decrementIntakeServoPos();
        }

        if(gamepad2.right_bumper) {
            startIntakeCRServo();
        } else if(gamepad2.left_bumper) {
            releaseSampleAsync();
        }
        updateTelemetry();
    }

    public void updateTelemetry() {
        this.telemetry.addLine();
        this.telemetry.addData("IntakeServo",
                String.format("InPos: %.3f, ServoPos: %.3f, ActPos:%.3f",
                        this.intakeServoPos,
                        this.intakeLiftServo.getPosition(),
                        getIntakeActPos()));
        this.telemetry.addData("IntakeDis", String.format("%.3f cm", this.intakeColorSensor.getDistance(DistanceUnit.CM)));
        this.telemetry.addData("IntakeColor", ColorUtility.getColorName(this.intakeColorSensor.getNormalizedColors()).name());
    }
}
