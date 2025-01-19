package org.firstinspires.ftc.teamcode.anime;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.CompletableFuture;
import java.util.logging.Logger;

public class Intake {

    private static Logger log = Logger.getLogger(Intake.class.getName());
    public static final double INTAKE_SERVO_START_POS = 30;
    public static final double INTAKE_SERVO_BACK_POS = 255;
    public static final double INTAKE_SERVO_FULL_BACK_POS = 300;
    public static final double INTAKE_SERVO_ATUO_POS = 0;

    public static final double CRSERVO_SAMPLE_INTAKE_POWER = -1.0;
    public static final double CRSERVO_SAMPLE_RELEASE_POWER = 1.0;
    public static final double CRSERVO_STOP_POWER = 0.0;

    private AnimeRobot robot;
    private HardwareMap hardwareMap;
    CRServo intakeLiftServo;
    CRServo intakeCRServo;
    RevColorSensorV3 intakeColorSensor;
    Telemetry telemetry;
    AnalogInput intakeServoAnalogInput, crServoAnalogInput;
    private CompletableFuture<Void> releaseSampleTask;
    private boolean gotSample = false;

    private CompletableFuture<Void> intakeLiftTask;

    public Intake(AnimeRobot robot) {
        this.robot = robot;
        this.hardwareMap = this.robot.getHardwareMap();
        this.intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        this.intakeLiftServo = hardwareMap.get(CRServo.class, "ils"); // Intake Lift Servo
        this.intakeCRServo = hardwareMap.get(CRServo.class, "crservo");
        this.intakeServoAnalogInput = hardwareMap.get(AnalogInput.class, "ilsai"); // Intake Lift Servo Actual Position
        this.crServoAnalogInput = hardwareMap.get(AnalogInput.class, "crsai"); // Continuous Servo Actual Position
        this.telemetry = this.robot.getOpMode().telemetry;

        this.intakeCRServo.setPower(CRSERVO_STOP_POWER);
    }

    public CRServo getIntakeLiftServo() {
        return intakeLiftServo;
    }

    public CRServo getIntakeCRServo() {
        return intakeCRServo;
    }

    public RevColorSensorV3 getIntakeColorSensor() {
        return intakeColorSensor;
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
        if(AsyncUtility.isTaskDone(intakeLiftTask)) {
            if(gamepad2.right_trigger > 0.5) {
                this.intakeLiftServo.setPower(0.5);
            } else if(gamepad2.left_trigger > 0.5) {
                this.intakeLiftServo.setPower(-0.5);
            } else {
                this.intakeLiftServo.setPower(0.0);
            }
        }

        if(gamepad2.right_bumper) {
            startIntakeCRServo();
        } else if(gamepad2.left_bumper) {
            releaseSampleAsync();
        }

        if(gamepad2.back) {
            setIntakeServoPosAsync(INTAKE_SERVO_BACK_POS, -1.0, 5);
        } else if(gamepad2.start) {
            setIntakeServoPosAsync(INTAKE_SERVO_START_POS, 1.0, 5);
        }

        double intakeSensorDistance = this.intakeColorSensor.getDistance(DistanceUnit.CM);
        if(!gotSample && intakeSensorDistance < 1) {
            intakeLiftGotTheSampleAsync();
            gotSample = true;
        } else if(gotSample && intakeSensorDistance > 1) {
            gotSample = false;
        }

        updateTelemetry();
    }

    public Runnable intakeLiftGotTheSample() {
        return () -> {
            this.intakeLiftServo.setPower(-1.0);
            this.robot.sleep(500);
            this.intakeLiftServo.setPower(0.0);
        };
    }

    public void intakeLiftGotTheSampleAsync() {
        this.intakeLiftTask = AsyncUtility.createAsyncTask(this.intakeLiftTask, intakeLiftGotTheSample());
    }

    public Runnable setIntakeServoPosTask(double pos, double power, double timeoutSec) {
        return () -> {
            double currentPos = getIntakeActPos();
            double prevDistance = 361;
            double distance = 360;
            boolean powerAdjusted = false;

            long timeout = System.currentTimeMillis() + (long) (timeoutSec * 1000);
            this.intakeLiftServo.setPower(power);
            log.info(String.format("IntakeServo set to pos: %.3f, power: %.3f, timeoutSec: %.3f", pos, power, timeoutSec));
            while(distance > 2 && (System.currentTimeMillis() < timeout)) {

                this.robot.sleep(50);
                currentPos = getIntakeActPos();
                distance = Math.abs(currentPos - pos);
                if(distance > 180 && pos > 300) {
                    distance = 360 - distance;
                }
//                log.info(String.format("IntakeServo currentPos: %.3f, distance: %.3f, prvDis: %.3f, timeLeft:%.3f, powerAdj: %b, power: %.3f",
//                        currentPos, distance, prevDistance, ((timeout-System.currentTimeMillis())/1000.0), powerAdjusted, this.intakeLiftServo.getPower()));
//                log.info(String.format("IntakeServo distance > 2: %b, (System.currentTimeMillis() < timeout): %b, distance < prevDistance: %b",
//                        distance > 2, (System.currentTimeMillis() < timeout), distance < prevDistance));
                if(distance > prevDistance) {
                    if(powerAdjusted) {
                        break;
                    } else {
                        this.intakeLiftServo.setPower(-power);
                        powerAdjusted = true;
                        prevDistance = 361;
                    }
                } else {
                    prevDistance = distance;
                }
            }
            this.intakeLiftServo.setPower(0);
            log.info(String.format("IntakeServo set to pos: %.3f, actPos:%.3f, power: %.3f, timeLeft: %.3f, done",
                    pos, getIntakeActPos(), power, ((timeout-System.currentTimeMillis())/1000.0)));
        };
    }

    public void setIntakeServoPosAsync(double pos, double power, double timeoutSec) {
        this.intakeLiftTask = AsyncUtility.createAsyncTask(this.intakeLiftTask, setIntakeServoPosTask(pos, power, timeoutSec));
    }

    public void updateTelemetry() {
        this.telemetry.addLine();
        this.telemetry.addData("IntakeServo",
                String.format("InPow: %.3f InPos: %.3f", this.intakeLiftServo.getPower(), getIntakeActPos()));
        this.telemetry.addData("IntakeDis", String.format("%.3f cm", this.intakeColorSensor.getDistance(DistanceUnit.CM)));
        this.telemetry.addData("IntakeColor", ColorUtility.getColorName(this.intakeColorSensor.getNormalizedColors()).name());
    }
}
