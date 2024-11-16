package org.firstinspires.ftc.teamcode.anime;

import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.LIFT_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.LIFT_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.MIN_LIFT_POWER;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MAX_LENGTH;
import static org.firstinspires.ftc.teamcode.anime.AnimeRobot.SLIDE_MIN_LENGTH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private AnimeRobot robot;
    public Lift(AnimeRobot robot) {
        this.robot = robot;
    }

    public double checkLiftPower(double liftPower) {
        if(liftPower > 0 && (robot.liftLeft.getCurrentPosition() > LIFT_MAX_HEIGHT || robot.liftRight.getCurrentPosition() > LIFT_MAX_HEIGHT)) {
            liftPower = MIN_LIFT_POWER;
        } else if (liftPower < 0 && (robot.liftLeft.getCurrentPosition() < LIFT_MIN_HEIGHT || robot.liftRight.getCurrentPosition() < LIFT_MIN_HEIGHT)) {
            liftPower = -MIN_LIFT_POWER;
        }
        return liftPower;
    }
}
