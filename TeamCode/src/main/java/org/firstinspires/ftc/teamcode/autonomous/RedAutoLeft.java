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
                        .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(45))
//                        .strafeTo(new Vector2d(41.5,-64))
//                        .setTangent(0)
//                        .splineTo(new Vector2d(-60, -60), Math.toRadians(45))
                        .build());
    }
}