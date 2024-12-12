package org.firstinspires.ftc.teamcode.anime;

import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_INIT_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_OUT_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_SPECIMEN_COLLECT_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_SPECIMEN_HANG_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.LIFT_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.LIFT_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.LIFT_SPECIMEN_HANG_HEIGHT;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.LIFT_SPECIMEN_PICK_UP_HEIGHT;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.MIN_LIFT_POWER;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.CompletableFuture;
import java.util.logging.Logger;

public class Lift {
    private static Logger log = Logger.getLogger(Lift.class.getName());
    private AnimeRobot robot;
    private CompletableFuture<Void> currentTask;

    private ElapsedTime liftRuntime = new ElapsedTime();

    public Lift(AnimeRobot robot) {
        this.robot = robot;
    }

    public double checkLiftPower(double liftPower) {
        if (liftPower > 0 && (robot.liftLeft.getCurrentPosition() > LIFT_MAX_HEIGHT || robot.liftRight.getCurrentPosition() > LIFT_MAX_HEIGHT)) {
            liftPower = MIN_LIFT_POWER;
        } else if (liftPower < 0 && (robot.liftLeft.getCurrentPosition() < LIFT_MIN_HEIGHT || robot.liftRight.getCurrentPosition() < LIFT_MIN_HEIGHT)) {
            liftPower = -MIN_LIFT_POWER;
        }
        return liftPower;
    }

    public void handleLiftManually(double manualPower) {
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }

        double liftPower = manualPower;
        liftPower = this.robot.getLift().checkLiftPower(liftPower);
        if (liftPower > 0) {
            this.robot.setHoldLift(true);
        } else if (liftPower == 0 && this.robot.isHoldLift()) {
            liftPower = MIN_LIFT_POWER;
        } else {
            this.robot.setHoldLift(false);
        }

        if (this.robot.hangTheBot) {
            liftPower = -1;
        }

        this.robot.liftLeft.setPower(liftPower);
        this.robot.liftRight.setPower(liftPower);
    }

    public void liftDown() {
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> liftDownTask());
    }

    public void liftUpToBasketLevel() {
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> liftUpToBasketTask());
    }

    public void specimenPickup() {
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> specimenPickupTask());
    }

    public void specimenDrop() {
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> specimenHangTask());
    }

    public Action liftDownAction() {
        return new AnimeAction(() -> liftDownTask());
    }

    public Action liftUpToBasketLevelAction() {
        return new AnimeAction(() -> liftUpToBasketTask());
    }

    public Action specimenPickupAction() {
        return new AnimeAction(() -> specimenPickupTask());
    }

    public Action specimenHangAction() {
        return new AnimeAction(() -> specimenHangTask());
    }

    public Action openDropClawAction() {
        return new AnimeAction(() -> {
            robot.setDropClawServoPos(DROP_CLAW_OPEN_POS);
            this.robot.sleep(200);
        });
    }

    public void dropServoOut() {
        CompletableFuture.runAsync(() -> {
            this.robot.sleep(200);
            dropServoOutTask();
        });
    }

    public void dropServoOutTask() {
        this.robot.setDropServoPos(DROP_SERVO_OUT_POS);
        this.robot.sleep(200);
    }

    public void liftDownTask() {
        this.robot.setDropClawServoPos(DROP_CLAW_OPEN_POS);
        this.robot.setDropServoPos(DROP_SERVO_INIT_POS);

        this.robot.liftLeft.setTargetPosition(-20);
        this.robot.liftRight.setTargetPosition(-20);
        this.robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.robot.liftLeft.setPower(1);
        this.robot.liftRight.setPower(1);

        liftRuntime.reset();
        while (this.robot.getOpMode().opModeIsActive() &&
                (liftRuntime.seconds() < 2.5) &&
                this.robot.liftLeft.isBusy() && this.robot.liftRight.isBusy()) {
            if (robot.liftLeft.getCurrentPosition() < LIFT_MIN_HEIGHT || robot.liftRight.getCurrentPosition() < LIFT_MIN_HEIGHT) {
                this.robot.liftLeft.setPower(MIN_LIFT_POWER);
                this.robot.liftRight.setPower(MIN_LIFT_POWER);
            }
            this.robot.getOpMode().idle();
        }
        this.robot.liftLeft.setPower(0);
        this.robot.liftRight.setPower(0);
        this.robot.setHoldLift(false);
        this.robot.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.robot.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void liftUpToBasketTask() {

        this.robot.setDropClawServoPos(DROP_CLAW_CLOSE_POS);

        int reducedSpeedPoint = LIFT_MAX_HEIGHT - 100;
        this.robot.liftLeft.setTargetPosition(LIFT_MAX_HEIGHT);
        this.robot.liftRight.setTargetPosition(LIFT_MAX_HEIGHT);
        this.robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.robot.liftLeft.setPower(1);
        this.robot.liftRight.setPower(1);

        liftRuntime.reset();
        this.dropServoOut(); // this with bring drop arm out
        while (this.robot.getOpMode().opModeIsActive() &&
                (liftRuntime.seconds() < 2.5) &&
                this.robot.liftLeft.isBusy() && this.robot.liftRight.isBusy()) {
            if (robot.liftLeft.getCurrentPosition() > reducedSpeedPoint || robot.liftRight.getCurrentPosition() > reducedSpeedPoint) {
                this.robot.liftLeft.setPower(MIN_LIFT_POWER);
                this.robot.liftRight.setPower(MIN_LIFT_POWER);
            }
            this.robot.getOpMode().idle();
        }

        this.robot.liftLeft.setPower(MIN_LIFT_POWER);
        this.robot.liftRight.setPower(MIN_LIFT_POWER);
        this.robot.setHoldLift(true);
        this.robot.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.robot.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void specimenPickupTask() {

        this.robot.liftLeft.setTargetPosition(LIFT_SPECIMEN_PICK_UP_HEIGHT);
        this.robot.liftRight.setTargetPosition(LIFT_SPECIMEN_PICK_UP_HEIGHT);
        this.robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.robot.liftLeft.setPower(1);
        this.robot.liftRight.setPower(1);

        liftRuntime.reset();
        while (this.robot.getOpMode().opModeIsActive() &&
                (liftRuntime.seconds() < 2.5) &&
                this.robot.liftLeft.isBusy() && this.robot.liftRight.isBusy()) {
            this.robot.getOpMode().idle();
        }

        this.robot.liftLeft.setPower(MIN_LIFT_POWER);
        this.robot.liftRight.setPower(MIN_LIFT_POWER);
        this.robot.setHoldLift(true);
        this.robot.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.robot.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.robot.setDropClawServoPos(DROP_CLAW_OPEN_POS);
        this.robot.setDropServoPos(DROP_SERVO_SPECIMEN_COLLECT_POS);
    }

    public void specimenHangTask() {

        this.robot.setDropClawServoPos(DROP_CLAW_CLOSE_POS);

        this.robot.liftLeft.setTargetPosition(LIFT_SPECIMEN_HANG_HEIGHT);
        this.robot.liftRight.setTargetPosition(LIFT_SPECIMEN_HANG_HEIGHT);
        this.robot.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.robot.liftLeft.setPower(1);
        this.robot.liftRight.setPower(1);

        liftRuntime.reset();
        while (this.robot.getOpMode().opModeIsActive() &&
                (liftRuntime.seconds() < 2.5) &&
                this.robot.liftLeft.isBusy() && this.robot.liftRight.isBusy()) {
            this.robot.getOpMode().idle();
        }

        this.robot.liftLeft.setPower(MIN_LIFT_POWER);
        this.robot.liftRight.setPower(MIN_LIFT_POWER);
        this.robot.setHoldLift(true);
        this.robot.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.robot.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.robot.setDropServoPos(DROP_SERVO_SPECIMEN_HANG_POS);
    }

    public void resetLift() {
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> resetLiftTask());
    }

    public void resetLiftTask() {
        this.robot.resetMotors("Lift", this.robot.liftLeft, this.robot.liftRight, liftRuntime);
    }
}
