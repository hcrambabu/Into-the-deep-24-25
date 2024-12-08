package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;

@Autonomous
public class RedAutoLeft extends BaseOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-24, -63, Math.PI / 2);
        Pose2d basetPose = new Pose2d(-52, -52, Math.toRadians(45));
        Pose2d sample1Pose = new Pose2d(-50, -50, Math.toRadians(90));
        this.initialize(beginPose);

        TrajectoryActionBuilder gotoBasket_1 = drive.actionBuilder(beginPose)
                .splineToLinearHeading(basetPose, Math.toRadians(180));
        TrajectoryActionBuilder gotoSample_1 = gotoBasket_1.endTrajectory().fresh()
                .splineToLinearHeading(sample1Pose, Math.toRadians(0));

        Action action = new SequentialAction(
                new ParallelAction(
                        gotoBasket_1.build()
                        , this.robot.getLift().goToDropAction()
                )
                , this.robot.getLift().openDropClawAction()
                , new ParallelAction(
                        gotoSample_1.build()
                        , new ParallelAction(
                    this.robot.getLift().goBackAction()
                            , this.robot.getIntake().intakeArmDownAction()
                        )
                )
        );

        waitForStart();
        Actions.runBlocking(action);
    }
}