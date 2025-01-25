package org.firstinspires.ftc.teamcode.anime;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

public abstract class BaseOpMode extends LinearOpMode {
    protected AnimeRobot robot;
    protected MecanumDrive drive;

    public void initialize(Pose2d beginPose, boolean isTeleOp) {
        // Initialize the robot
        this.robot = new AnimeRobot(this, beginPose, isTeleOp);
        this.drive = this.robot.getDrive();
    }
}
