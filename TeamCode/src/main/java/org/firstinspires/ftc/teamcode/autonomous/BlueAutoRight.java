package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;

@Autonomous
public class BlueAutoRight extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-24, 61, Math.toRadians(270));
        this.initialize(beginPose);

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d( -44, 12),Math.PI/2)
                        //push first block
                        .lineToY(55)
                        .lineToY(10)
                        //go left
                        .strafeTo(new Vector2d(-54, 10))
                        //push block #2
                        .strafeTo(new Vector2d(-54, 54))
                        .lineToY(10)
                        //position for final block
                        .strafeTo(new Vector2d(-61, 10))
                        .strafeTo(new Vector2d(-61, 54))
                        .build());
    }
}
