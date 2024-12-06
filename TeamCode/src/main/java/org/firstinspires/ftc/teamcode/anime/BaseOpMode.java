package org.firstinspires.ftc.teamcode.anime;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

public abstract class BaseOpMode extends LinearOpMode {
    protected AnimeRobot robot;
    protected PinpointDrive drive;

    public void initialize(Pose2d beginPose) {
        // Initialize the robot
        this.robot = new AnimeRobot(this, beginPose);
        this.drive = this.robot.getDrive();
    }
}
