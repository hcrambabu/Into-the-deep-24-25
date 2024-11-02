package org.firstinspires.ftc.teamcode.anime;

import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MAX_LENGTH;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MIN_LENGTH;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private AnimeRobot robot;
    public Intake(AnimeRobot robot) {
        this.robot = robot;
    }

    public double checkSlidePower(double slidePower) {
//        if(slidePower > 0 && (robot.slideLeft.getCurrentPosition() > SLIDE_MAX_LENGTH || robot.slideRight.getCurrentPosition() > SLIDE_MAX_LENGTH)) {
//            slidePower = AnimeRobot.MIN_SLIDE_POWER;
//        } else if(slidePower < 0 && (robot.slideLeft.getCurrentPosition() < SLIDE_MIN_LENGTH || robot.slideRight.getCurrentPosition() < SLIDE_MIN_LENGTH)) {
//            slidePower = -AnimeRobot.MIN_SLIDE_POWER;
//        }
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
}
