package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;

@Autonomous
public class RedAutoRight extends BaseOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-24, -63, Math.toRadians(90));
        this.initialize(beginPose);

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
//                        .setTangent(0)
//                        .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(90)), Math.toRadians(0))
//                        .setTangent(Math.toRadians(180))
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(90)), Math.toRadians(0))
//                        .setTangent(Math.toRadians(270))
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), Math.toRadians(180))
                        .build());
    }
}