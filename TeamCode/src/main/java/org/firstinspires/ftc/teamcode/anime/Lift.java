package org.firstinspires.ftc.teamcode.anime;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.concurrent.CompletableFuture;
import java.util.function.IntSupplier;
import java.util.logging.Logger;

public class Lift {

    public static final int LOWER_LIFT_MIN_POS = 0;
    public static final int LOWER_LIFT_UP_POS = 2106;
    public static final int LOWER_LIFT_DOWN_POS = 500;
    public static final int LOWER_LIFT_SPECIMEN_PICKUP_POS = 672;
    public static final int LOWER_LIFT_SPECIMEN_DROP_POS = 1510;

    public static final double LOWER_LIFT_MIN_POWER = 0.05;
    public static final int UPPER_LIFT_MIN_POS = 0;
    public static final int UPPER_LIFT_UP_POS = 750;
    public static final int UPPER_LIFT_DOWN_POS = 2000;
    public static final int UPPER_LIFT_SPECIMEN_PICKUP_POS = 878;
    public static final int UPPER_LIFT_SPECIMEN_DROP_POS = 864;

    private static Logger log = Logger.getLogger(Lift.class.getName());
    private AnimeRobot robot;
    private HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotorEx lowerLiftMotor, upperLiftMotor;
    RevColorSensorV3 specimenColorSensor;
    private CompletableFuture<Void> lowerLiftTask;
    private CompletableFuture<Void> upperLiftTask;

    private ElapsedTime resetTimer = new ElapsedTime();

    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    public Lift(AnimeRobot robot) {
        this.robot = robot;
        this.telemetry = this.robot.getOpMode().telemetry;
        this.hardwareMap = this.robot.getHardwareMap();

        this.specimenColorSensor = hardwareMap.get(RevColorSensorV3.class, "specimenColor");
        this.lowerLiftMotor = hardwareMap.get(DcMotorEx.class, "llm");
        this.upperLiftMotor = hardwareMap.get(DcMotorEx.class, "ulm");

        this.lowerLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        //this.upperLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        this.lowerLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.upperLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetLift();
        this.lowerLiftMotor.setMode(runMode);
        this.upperLiftMotor.setMode(runMode);
    }

    public void resetLift() {
        resetMotor("Lower", lowerLiftMotor);
        resetMotor("Upper", upperLiftMotor);
    }

    public void resetMotor(String name,DcMotorEx motor) {
        motor.setPower(-0.4);
        try {
            Thread.sleep(200);
        } catch (InterruptedException ex) {
        } // Wait little time for motors to start
        resetTimer.reset();
        log.info(String.format("Motor {%s} reset start... velocity {%.3f}", name, motor.getVelocity()));
        while (resetTimer.seconds() < 5 && Math.abs(motor.getVelocity()) > 400) {
            Thread.yield();
            log.info(String.format("In resetMotors {%s} ... Waiting for zero velocity ... velocity {%.3f}", name, motor.getVelocity()));
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);
        log.info(String.format("Motor {%s} reset complete", name));
    }

    public RevColorSensorV3 getSpecimenColorSensor() {
        return specimenColorSensor;
    }

    public int getActualLowerLiftPos() {
        return lowerLiftMotor.getCurrentPosition();
    }

    public int getActualUpperLiftPos() {
        return upperLiftMotor.getCurrentPosition();
    }

    public void setLowerLiftPower(double power) {
        this.lowerLiftMotor.setPower(power);
        //this.setUpperLiftPower(power/2);
    }

    public void setUpperLiftPower(double power) {
        this.upperLiftMotor.setPower(power);
    }

    public void holdLowerLift() {
        if(this.getActualLowerLiftPos() < LOWER_LIFT_UP_POS) {
            this.setLowerLiftPower(LOWER_LIFT_MIN_POWER);
        } else {
            this.setLowerLiftPower(-LOWER_LIFT_MIN_POWER);
        }
    }

    public void handleKeyPress(Gamepad gamepad1, Gamepad gamepad2) {

        if(AsyncUtility.isTaskDone(upperLiftTask)) {
            if (gamepad2.left_stick_y == 0) { // set minimum power to keep lift in position
                holdLowerLift();
            } else {
                this.setLowerLiftPower(-gamepad2.left_stick_y);
            }
        }

        if(AsyncUtility.isTaskDone(lowerLiftTask)) {
            this.setUpperLiftPower(gamepad2.right_stick_x);
        }

        if (gamepad2.back) {
            liftUp();
        } else if (gamepad2.start) {
            liftDown();
        } else if (gamepad2.a) {
            liftSpecimenPickup();
        } else if (gamepad2.b) {
            liftSpecimenDROP();
        }

        updateTelemetry();
    }

    public void updateTelemetry() {
        this.telemetry.addLine();
        this.telemetry.addData("lift power", "lower: %.3f, upper: %.3f", this.lowerLiftMotor.getPower(), this.upperLiftMotor.getPower());
        this.telemetry.addData("lift Act pos", "lower: %d, upper: %d", this.getActualLowerLiftPos(), this.getActualUpperLiftPos());
        this.telemetry.addData("specimenDis", String.format("%.3f cm", this.specimenColorSensor.getDistance(DistanceUnit.CM)));
        this.telemetry.addData("specimenColor", ColorUtility.getColorName(this.specimenColorSensor.getNormalizedColors()).name());
    }

    public void gotoPosition(String name, DcMotorEx motor, int pos, double timeoutSec) {
        int numTries = 0;
        while(numTries < 2 && Math.abs(motor.getCurrentPosition() - pos) > 10) {
            numTries++;
            motor.setTargetPosition(pos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
            long timeout = System.currentTimeMillis() + (long) (timeoutSec * 1000);
            while (motor.isBusy() && System.currentTimeMillis() < timeout) {
                motor.setPower(1);
                Thread.yield();
            }
            if ("lower".equals(name)) {
                holdLowerLift();
            } else {
                motor.setPower(0);
            }
            log.info(String.format("Motor {%s} goToPositionTask complete.. {%d} -- {%d} -- time diff: {%d}",
                    name, pos, motor.getCurrentPosition(), (long) (timeout - System.currentTimeMillis())));

            motor.setMode(runMode);
        }
    }

    public Runnable goToPositionTask(String name, DcMotorEx motor, int pos, double timeoutSec) {
        return () -> {
            gotoPosition(name, motor, pos, timeoutSec);
        };
    }

    public void liftUp() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_UP_POS, 5));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_UP_POS, 5));
    }

    public Action liftUpAction() {
        return liftAction(LOWER_LIFT_UP_POS, UPPER_LIFT_UP_POS);
    }

    public void liftDown() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_DOWN_POS, 5));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_DOWN_POS, 5));
    }

    public Action liftDownAction() {
        return liftAction(LOWER_LIFT_DOWN_POS, UPPER_LIFT_DOWN_POS);
    }

    public void liftHome() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_MIN_POS, 5));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_MIN_POS, 5));
    }

    public void liftSpecimenPickup() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_SPECIMEN_PICKUP_POS, 5));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_SPECIMEN_PICKUP_POS, 5));
    }

    public void liftSpecimenDROP() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_SPECIMEN_DROP_POS, 5));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_SPECIMEN_DROP_POS, 5));
    }

    public Action liftHomeAction() {
        return liftAction(LOWER_LIFT_MIN_POS, UPPER_LIFT_MIN_POS);
    }

    public Action liftAction(int lowePos, int upperPos) {
        return new ParallelAction(
                AnimeAction.createAction(goToPositionTask("lower", lowerLiftMotor, lowePos, 5)),
                AnimeAction.createAction(goToPositionTask("upper", upperLiftMotor, upperPos, 5))
        );
    }

    public Action resetLiftAction() {
        return AnimeAction.createAction(this::resetLift);
    }

    public void cancelCurrentTask() {
        if (lowerLiftTask != null) {
            lowerLiftTask.cancel(true);
        }
        if (upperLiftTask != null) {
            upperLiftTask.cancel(true);
        }
    }
}
