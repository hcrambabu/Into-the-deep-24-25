package org.firstinspires.ftc.teamcode.anime;

import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_0;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.INTAKE_LIFT_DOWN_POS_1;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MAX_LENGTH;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MIN_LENGTH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private AnimeRobot robot;
    private ElapsedTime intakeRuntime = new ElapsedTime();
    public Intake(AnimeRobot robot) {
        this.robot = robot;
    }

    public double checkSlidePower(double slidePower) {
        if(slidePower > 0 && (robot.slideLeft.getCurrentPosition() > SLIDE_MAX_LENGTH || robot.slideRight.getCurrentPosition() > SLIDE_MAX_LENGTH)) {
            slidePower = AnimeRobot.MIN_SLIDE_POWER;
        } else if(slidePower < 0 && (robot.slideLeft.getCurrentPosition() < SLIDE_MIN_LENGTH || robot.slideRight.getCurrentPosition() < SLIDE_MIN_LENGTH)) {
            slidePower = -AnimeRobot.MIN_SLIDE_POWER;
        }
        return slidePower;
    }
    public void slideOut() {
        double slidePower = 1;
        slidePower = checkSlidePower(slidePower);

        robot.slideLeft.setPower(slidePower);
        robot.slideRight.setPower(slidePower);
    }

    public void slideIn() {
        double slidePower = -1;
        slidePower = checkSlidePower(slidePower);

        robot.slideLeft.setPower(slidePower);
        robot.slideRight.setPower(slidePower);
    }

    public void goBack() {
        this.robot.intakeClawServo.setPosition(0.0);
        this.robot.intakeVerticalTurnServo.setPosition(AnimeRobot.INTAKE_V_BACK_POS);
        this.robot.intakeHorizontalTurnServo.setPosition(AnimeRobot.INTAKE_H_H_POS);
        this.robot.intakeLiftServo.setPosition(INTAKE_LIFT_DOWN_POS_0);

        this.robot.slideLeft.setTargetPosition(-20);
        this.robot.slideRight.setTargetPosition(-20);
        this.robot.slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.robot.slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.robot.slideLeft.setPower(1);
        this.robot.slideRight.setPower(1);

        intakeRuntime.reset();
        while(this.robot.getOpMode().opModeIsActive() &&
                (intakeRuntime.seconds() < 1.5) &&
                this.robot.slideLeft.isBusy() && this.robot.slideRight.isBusy()) {
            this.robot.getOpMode().idle();
        }

        this.robot.slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.robot.slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void goFront() {
        this.robot.intakeClawServo.setPosition(1.0);
        this.robot.intakeVerticalTurnServo.setPosition(1.0);
        this.robot.intakeHorizontalTurnServo.setPosition(AnimeRobot.INTAKE_H_H_POS);
        this.robot.intakeLiftServo.setPosition(INTAKE_LIFT_DOWN_POS_1);
    }
}
