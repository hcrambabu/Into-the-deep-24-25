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
    public static final int LOWER_LIFT_MAX_POS = 4000;
    public static final int LOWER_LIFT_UP_POS = 2106;
    public static final int LOWER_LIFT_DOWN_POS = 500;
    public static final int LOWER_LIFT_DOWN_POS_2 = 63;
    public static final int LOWER_LIFT_SPECIMEN_PICKUP_POS = 600;
    public static final int LOWER_LIFT_SPECIMEN_DROP_POS = 1473;
    public static final int LOWER_LIFT_SPECIMEN_DROP_POS_2 = 1000;
    public static final int LOWER_LIFT_INCREMENT = 100;
    public static final double LOWER_LIFT_MIN_POWER = 0.05;

    public static final int UPPER_LIFT_MIN_POS = 0;
    public static final int UPPER_LIFT_MAX_POS = 4000;
    public static final int UPPER_LIFT_UP_POS = 750;
    public static final int UPPER_LIFT_DOWN_POS = 2000;
    public static final int UPPER_LIFT_DOWN_POS_2 = 1337;
    public static final int UPPER_LIFT_SPECIMEN_PICKUP_POS = 878;
    public static final int UPPER_LIFT_SPECIMEN_DROP_POS = 860;
    public static final int UPPER_LIFT_SPECIMEN_DROP_POS_2 = 764;
    public static final int UPPER_LIFT_INCREMENT = 150;

    public static final double RUN_TO_POS_TIMEOUT = 2;

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

    private boolean isTeleOp = false;

    private int lowerLiftPos = LOWER_LIFT_MIN_POS;
    private int upperLiftPos = UPPER_LIFT_MIN_POS;

    public Lift(AnimeRobot robot, boolean isTeleOp) {
        this.isTeleOp = isTeleOp;
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
//        if(!isTeleOp) {
            this.runMode = DcMotor.RunMode.RUN_TO_POSITION;
            this.lowerLiftMotor.setTargetPosition(LOWER_LIFT_MIN_POS);
            this.upperLiftMotor.setTargetPosition(UPPER_LIFT_MIN_POS);
//        }
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
        while (resetTimer.seconds() < 5 && Math.abs(motor.getVelocity()) > 300) {
            Thread.yield();
            log.info(String.format("In resetMotors {%s} ... Waiting for zero velocity ... velocity {%.3f}", name, motor.getVelocity()));
        }

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        //this.lowerLiftMotor.setPower(power);
        this.lowerLiftPos += (int)(LOWER_LIFT_INCREMENT * power);
//        if(this.lowerLiftPos < LOWER_LIFT_MIN_POS) {
//            this.lowerLiftPos = LOWER_LIFT_MIN_POS;
//        } else if(this.lowerLiftPos > LOWER_LIFT_MAX_POS) {
//            this.lowerLiftPos = LOWER_LIFT_MAX_POS;
//        }
        this.lowerLiftMotor.setTargetPosition(this.lowerLiftPos);
    }

    public void setUpperLiftPower(double power) {
//        this.upperLiftMotor.setPower(power);
        this.upperLiftPos += (int)(UPPER_LIFT_INCREMENT * power);
//        if(this.upperLiftPos < UPPER_LIFT_MIN_POS) {
//            this.upperLiftPos = UPPER_LIFT_MIN_POS;
//        } else if(this.upperLiftPos > UPPER_LIFT_MAX_POS) {
//            this.upperLiftPos = UPPER_LIFT_MAX_POS;
//        }
        this.upperLiftMotor.setTargetPosition(this.upperLiftPos);
    }

    public void holdLowerLift() {
        this.setLowerLiftPower(getLowerLiftHoldPower(this.getActualLowerLiftPos()));
    }

    public double getLowerLiftHoldPower(int pos) {
        if(this.runMode == DcMotor.RunMode.RUN_TO_POSITION) {
            return 1;
        }

        if(pos < LOWER_LIFT_UP_POS) {
            return LOWER_LIFT_MIN_POWER;
        } else {
            return -LOWER_LIFT_MIN_POWER;
        }
    }

    public double getUpperHoldPower(int pos) {
        if(this.runMode == DcMotor.RunMode.RUN_TO_POSITION) {
            return 1;
        }
        return 0;
    }

    public void handleKeyPress(Gamepad gamepad1, Gamepad gamepad2) {

        if(AsyncUtility.isTaskDone(upperLiftTask)) {
//            if (gamepad2.left_stick_y == 0) { // set minimum power to keep lift in position
//                holdLowerLift();
//            } else {
//                this.setLowerLiftPower(-gamepad2.left_stick_y);
//            }
            this.setLowerLiftPower(-gamepad2.left_stick_y);
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
        } else if (gamepad2.x) {
            liftDown_2();
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
        while(numTries < 1 && Math.abs(motor.getCurrentPosition() - pos) > 10) {
            numTries++;

            double endPower = getUpperHoldPower(pos);
            if ("lower".equals(name)) {
                endPower = getLowerLiftHoldPower(pos);
                this.lowerLiftPos = pos;
            } else {
                this.upperLiftPos = pos;
            }

            motor.setTargetPosition(pos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
            long timeout = System.currentTimeMillis() + (long) (timeoutSec * 1000);
            while (motor.isBusy() && System.currentTimeMillis() < timeout) {
                motor.setPower(1);
                Thread.yield();
            }
            motor.setPower(endPower);
            motor.setMode(runMode);

            log.info(String.format("Motor {%s} goToPositionTask complete.. {%d} -- {%d} -- time diff: {%d}",
                    name, pos, motor.getCurrentPosition(), (long) (timeout - System.currentTimeMillis())));
        }
    }

    public Runnable goToPositionTask(String name, DcMotorEx motor, int pos, double timeoutSec) {
        return () -> {
            gotoPosition(name, motor, pos, timeoutSec);
        };
    }

    public void liftUp() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_UP_POS, RUN_TO_POS_TIMEOUT));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_UP_POS, RUN_TO_POS_TIMEOUT));
    }

    public Action liftUpAction() {
        return liftAction(LOWER_LIFT_UP_POS, UPPER_LIFT_UP_POS);
    }

    public void liftDown() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_DOWN_POS, RUN_TO_POS_TIMEOUT));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_DOWN_POS, RUN_TO_POS_TIMEOUT));
    }

    public void liftDown_2() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_DOWN_POS_2, RUN_TO_POS_TIMEOUT));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_DOWN_POS_2, RUN_TO_POS_TIMEOUT));
    }

    public Action liftDownAction() {
        return liftAction(LOWER_LIFT_DOWN_POS, UPPER_LIFT_DOWN_POS);
    }

    public void liftHome() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_MIN_POS, RUN_TO_POS_TIMEOUT));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_MIN_POS, RUN_TO_POS_TIMEOUT));
    }

    public void liftSpecimenPickup() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_SPECIMEN_PICKUP_POS, RUN_TO_POS_TIMEOUT));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_SPECIMEN_PICKUP_POS, RUN_TO_POS_TIMEOUT));
    }

    public void liftSpecimenDROP() {
        this.lowerLiftTask = AsyncUtility.createAsyncTask(lowerLiftTask, goToPositionTask("lower", lowerLiftMotor, LOWER_LIFT_SPECIMEN_DROP_POS, RUN_TO_POS_TIMEOUT));
        this.upperLiftTask = AsyncUtility.createAsyncTask(upperLiftTask, goToPositionTask("upper", upperLiftMotor, UPPER_LIFT_SPECIMEN_DROP_POS, RUN_TO_POS_TIMEOUT));
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
