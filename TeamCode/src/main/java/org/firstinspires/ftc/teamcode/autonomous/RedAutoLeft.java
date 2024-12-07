package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@Autonomous
public class RedAutoLeft extends BaseOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-24, -63, Math.PI / 2);
        this.initialize(beginPose);

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        .build());
    }
}