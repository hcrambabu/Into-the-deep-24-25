package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;

@Autonomous(group = "Anime", name = "Right")
public class RedAutoRight extends BaseOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(24, -63, Math.PI/2);
        this.initialize(beginPose);

        VelConstraint pushVelConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 10;
            }
        };

        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d( 48, -12),Math.toRadians(-90))
                .strafeTo(new Vector2d(46, -12))
                .strafeTo(new Vector2d(46, -56))
                .strafeTo(new Vector2d(46, -12))
                .strafeTo(new Vector2d(58, -12))
                .strafeTo(new Vector2d(58, -56))
                .lineToY(-12)
                .strafeTo(new Vector2d(63, -12))
                .strafeTo(new Vector2d(63, -52), pushVelConstraint);
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        this.robot.getIntake().CloseClawAction(),
                        trajectoryActionBuilder.build(),
                        this.robot.getIntake().trunFaceAndOpenClawAction(),
                        new SleepAction(2)
                )
        );
    }
}