package org.firstinspires.ftc.teamcode.anime;

import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_INIT_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_FACE_DOWN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_UP_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_1;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_FULL_DOWN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_HUSKYLENS_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.MIN_SLIDE_POWER;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MAX_LENGTH;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MIN_LENGTH;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.CompletableFuture;

public class Intake {

    public static final double ticksPerInch = 2050/24;

    private AnimeRobot robot;

    private CompletableFuture<Void> currentTask;
    private ElapsedTime intakeRuntime = new ElapsedTime();

    private CompletableFuture<Void> goBacktask;
    private CompletableFuture<Void> liftUpToBasketLevelTask;

    public Intake(AnimeRobot robot) {
        this.robot = robot;
    }

    public double checkSlidePower(double slidePower) {
        if (slidePower > 0 && (robot.slideLeft.getCurrentPosition() > SLIDE_MAX_LENGTH || robot.slideRight.getCurrentPosition() > SLIDE_MAX_LENGTH)) {
            slidePower = MIN_SLIDE_POWER;
        } else if (slidePower < 0 && (robot.slideLeft.getCurrentPosition() < SLIDE_MIN_LENGTH || robot.slideRight.getCurrentPosition() < SLIDE_MIN_LENGTH)) {
            slidePower = -MIN_SLIDE_POWER;
        }
        return slidePower;
    }

    public void handleSlideManually(double manualPower) {
        if (currentTask != null && !currentTask.isDone() && !currentTask.isCancelled()) {
            return;
        }

        double slidePower = manualPower;
        slidePower = this.robot.getIntake().checkSlidePower(slidePower);

        this.robot.slideLeft.setPower(slidePower);
        this.robot.slideRight.setPower(slidePower);
    }

    public void goBack() {
        if (currentTask != null && !currentTask.isDone() && !currentTask.isCancelled()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> goBackTask(true));
    }

    public void goFront(boolean cancelGoBackTask) {
        if(cancelGoBackTask && goBacktask != null && !goBacktask.isDone() && !goBacktask.isCancelled()) {
            goBacktask.cancel(true);
        }
        if(cancelGoBackTask && liftUpToBasketLevelTask != null && !liftUpToBasketLevelTask.isDone() && !liftUpToBasketLevelTask.isCancelled()) {
            liftUpToBasketLevelTask.cancel(true);
        }
        if (currentTask != null && !currentTask.isDone() && !currentTask.isCancelled()) {
            return;
        }

        this.currentTask = CompletableFuture.runAsync(() -> goFrontTask()); //goFrontTask  catchSampleTask
        goBacktask = currentTask;
    }

    public Action goFrontAction() {
        return new AnimeAction(() -> goFrontTask());
    }

    public Action goBackAction() {
        return new AnimeAction(() -> goBackTask(false));
    }

    public Action intakeArmDownAction() {
        return new AnimeAction(() -> {
            robot.setIntakeLiftServoPos(INTAKE_LIFT_FULL_DOWN_POS);
            this.robot.sleep(200);
        });
    }
    public Action intakeArmDownToHuskyAction() {
        return new AnimeAction(() -> {
            robot.setIntakeLiftServoPos(INTAKE_LIFT_HUSKYLENS_POS);
            this.robot.sleep(200);
        });
    }

    public Action OpenClawAction() {
        return new AnimeAction(() -> {
            robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
            this.robot.sleep(200);
        });
    }

    public Action CloseClawAction() {
        return new AnimeAction(() -> {
            robot.setIntakeClawServoPos(INTAKE_CLAW_CLOSE_POS);
        });
    }

    public Action trunFaceAndOpenClawAction() {
        return new AnimeAction(() -> {
            robot.setIntakeLiftServoPos(INTAKE_LIFT_DOWN_POS_1);
            robot.setIntakeFaceUpDownServoPos(INTAKE_FACE_DOWN_POS);
            this.robot.sleep(200);
            robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
            this.robot.sleep(200);
        });
    }

    public Action catchSampleAction() {
        return new AnimeAction(() -> {
            catchSampleTask();
        });
    }

    public void catchSampleTask() {
        robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
        this.robot.setIntakeHorizontalTurnServoPos(AnimeRobot.INTAKE_ROTATING_SERVO_HORIZANTAL_POS);
        robot.setIntakeFaceUpDownServoPos(INTAKE_FACE_DOWN_POS);
        this.robot.sleep(500);
        robot.setIntakeLiftServoPos(INTAKE_LIFT_FULL_DOWN_POS);
        this.robot.sleep(400);
        robot.setIntakeClawServoPos(INTAKE_CLAW_CLOSE_POS);
        this.robot.sleep(400);
    }

    public void goBackTask(boolean moveLiftUp) {
        this.robot.setIntakeFaceUpDownServoPos(AnimeRobot.INTAKE_FACE_UP_POS);
        this.robot.setDropServoPos(DROP_SERVO_INIT_POS);
        this.robot.setDropClawServoPos(DROP_CLAW_OPEN_POS);
        this.robot.setIntakeClawServoPos(INTAKE_CLAW_CLOSE_POS);
        this.robot.setIntakeHorizontalTurnServoPos(AnimeRobot.INTAKE_ROTATING_SERVO_HORIZANTAL_POS);
        this.robot.sleep(300);
        this.robot.setIntakeLiftServoPos(INTAKE_LIFT_UP_POS);

        this.robot.slideLeft.setTargetPosition(-20);
        this.robot.slideRight.setTargetPosition(-20);
        this.robot.slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.robot.slideLeft.setPower(1);
        this.robot.slideRight.setPower(1);

        intakeRuntime.reset();
        while (this.robot.getOpMode().opModeIsActive() &&
                (intakeRuntime.seconds() < 1.5) &&
                this.robot.slideLeft.isBusy() && this.robot.slideRight.isBusy()) {
            if (robot.slideLeft.getCurrentPosition() < SLIDE_MIN_LENGTH || robot.slideRight.getCurrentPosition() < SLIDE_MIN_LENGTH) {
                this.robot.slideLeft.setPower(MIN_SLIDE_POWER);
                this.robot.slideRight.setPower(MIN_SLIDE_POWER);
            }
            this.robot.getOpMode().idle();
        }
        this.robot.slideLeft.setPower(0);
        this.robot.slideRight.setPower(0);
        this.robot.slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.robot.slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.robot.sleep(200); // Wait for lift to go up
        // Close Drop Claw and Open Intake Claw
        this.robot.setDropClawServoPos(DROP_CLAW_CLOSE_POS);
        this.robot.sleep(200);// wait for drop claw to close
        this.robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
        this.robot.sleep(100);
        if(moveLiftUp) {
            liftUpToBasketLevelTask = this.robot.getLift().liftUpToBasketLevel(true);
        }
    }

    public void goFrontTask() {
        this.robot.setDropServoPos(DROP_SERVO_INIT_POS);
        this.robot.setDropClawServoPos(DROP_CLAW_OPEN_POS);
        this.robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
        this.robot.setIntakeFaceUpDownServoPos(INTAKE_FACE_DOWN_POS);
        this.robot.setIntakeHorizontalTurnServoPos(AnimeRobot.INTAKE_ROTATING_SERVO_HORIZANTAL_POS);
        this.robot.setIntakeLiftServoPos(INTAKE_LIFT_DOWN_POS_1);
    }

    public void resetIntake() {
        if (currentTask != null && !currentTask.isDone() && !currentTask.isCancelled()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> resetIntakeTask());
    }

    public void resetIntakeTask() {
        this.robot.resetMotors("Intake", this.robot.slideLeft, this.robot.slideRight, intakeRuntime);
    }

    public void searchForSampleTask(double timeout, double maxSlideInches) {
        Pose2d blockPose = searchForSampleUsingSlide(timeout, maxSlideInches);
    }

    public Pose2d searchForSampleUsingSlide(double timeout, double maxSlideInches) {
        int ticks = SLIDE_MAX_LENGTH;
        if(maxSlideInches != -1) {
            ticks = (int) (maxSlideInches * ticksPerInch);
        }

        Pose2d blockPose = null;
        robot.setIntakeLiftServoPos(INTAKE_LIFT_UP_POS);

        this.robot.slideLeft.setTargetPosition(ticks);
        this.robot.slideRight.setTargetPosition(ticks);
        this.robot.slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.robot.slideLeft.setPower(1.0);
        this.robot.slideRight.setPower(1.0);

        intakeRuntime.reset();
        while (this.robot.getOpMode().opModeIsActive() &&
                (intakeRuntime.seconds() < timeout) &&
                this.robot.slideLeft.isBusy() && this.robot.slideRight.isBusy()) {
            blockPose = this.robot.getIntakeHusky().getBlockPoseWrtRobot();
            if(blockPose != null && blockPose.position.y < 13.5) {
                break;
            }
            this.robot.getOpMode().idle();
        }
        this.robot.slideLeft.setPower(0);
        this.robot.slideRight.setPower(0);
        this.robot.slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.robot.slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return blockPose;
    }

    public Action searchForSampleAction(double timeout, double maxSlideInches) {
        return new AnimeAction(() -> searchForSampleTask(timeout, maxSlideInches));
    }

    public void searchForSample(double timeout, double maxSlideInches) {
        if (currentTask != null && !currentTask.isDone() && !currentTask.isCancelled()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> searchForSampleTask(timeout, maxSlideInches));
    }
}
