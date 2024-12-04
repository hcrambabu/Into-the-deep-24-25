package org.firstinspires.ftc.teamcode.anime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseOpMode extends LinearOpMode {
    protected AnimeRobot robot;

    public void initialize() {
        // Initialize the robot
        this.robot = new AnimeRobot(this);
    }
}
