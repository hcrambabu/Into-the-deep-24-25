package org.firstinspires.ftc.teamcode.anime;

import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.DROP_SERVO_INIT_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_FACE_DOWN_POS;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_0;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_1;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_2;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.MIN_SLIDE_POWER;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MAX_LENGTH;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MIN_LENGTH;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.CompletableFuture;

public class Intake {
    private AnimeRobot robot;

    private CompletableFuture<Void> currentTask;
    private ElapsedTime intakeRuntime = new ElapsedTime();

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
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }

        double slidePower = manualPower;
        slidePower = this.robot.getIntake().checkSlidePower(slidePower);

        this.robot.slideLeft.setPower(slidePower);
        this.robot.slideRight.setPower(slidePower);
    }

    public void goBack() {
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> goBackTask());
    }

    public void goFront() {
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> goFrontTask());
    }

    public Action goFrontAction() {
        return telemetryPacket -> {
            goFrontTask();
            return false;
        };
    }

    public Action goBackAction() {
        return telemetryPacket -> {
            goBackTask();
            return false;
        };
    }

    public Action intakeArmDownAction() {
        return telemetryPacket -> {
            robot.setIntakeLiftServoPos(INTAKE_LIFT_DOWN_POS_1);
            return false;
        };
    }

    public Action catchSample() {
        return telemetryPacket -> {
            robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
            this.robot.setIntakeHorizontalTurnServoPos(AnimeRobot.INTAKE_HORIZANTAL_POS);
            robot.setIntakeFaceUpDownServoPos(INTAKE_FACE_DOWN_POS);
            robot.setIntakeLiftServoPos(INTAKE_LIFT_DOWN_POS_2);
            this.robot.sleep(200);
            robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
            this.robot.sleep(200);
            return false;
        };
    }


    public void goBackTask() {
        this.robot.setDropClawServoPos(DROP_CLAW_OPEN_POS);
        this.robot.setDropServoPos(DROP_SERVO_INIT_POS);
        this.robot.setIntakeClawServoPos(INTAKE_CLAW_CLOSE_POS);
        this.robot.setIntakeFaceUpDownServoPos(AnimeRobot.INTAKE_FACE_UP_POS);
        this.robot.setIntakeHorizontalTurnServoPos(AnimeRobot.INTAKE_HORIZANTAL_POS);
        this.robot.setIntakeLiftServoPos(INTAKE_LIFT_DOWN_POS_0);

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
        this.robot.sleep(400);// wait for drop claw to close
        this.robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
        this.robot.sleep(100);
    }

    public void goFrontTask() {
        this.robot.setDropServoPos(DROP_SERVO_INIT_POS);
        this.robot.setDropClawServoPos(DROP_CLAW_OPEN_POS);
        this.robot.setIntakeClawServoPos(INTAKE_CLAW_OPEN_POS);
        this.robot.setIntakeFaceUpDownServoPos(INTAKE_FACE_DOWN_POS);
        this.robot.setIntakeHorizontalTurnServoPos(AnimeRobot.INTAKE_HORIZANTAL_POS);
        this.robot.setIntakeLiftServoPos(INTAKE_LIFT_DOWN_POS_1);
    }

    public void resetIntake() {
        if (currentTask != null && !currentTask.isDone()) {
            return;
        }
        this.currentTask = CompletableFuture.runAsync(() -> resetIntakeTask());
    }

    public void resetIntakeTask() {
        this.robot.resetMotors("Intake", this.robot.slideLeft, this.robot.slideRight, intakeRuntime);
    }
}
