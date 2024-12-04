package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;

@Autonomous
public class BlueAutoRight extends BaseOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        this.initialize();
        Pose2d beginPose = new Pose2d(-24, 63, -Math.PI / 2);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(-65.5, 64))
                        .build());
    }
}
