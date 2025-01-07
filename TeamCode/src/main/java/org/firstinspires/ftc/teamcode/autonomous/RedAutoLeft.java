package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;

@Autonomous(group = "Anime", name = "Left")
//@TeleOp(group = "Anime", name = "Anime: RedLeft")
public class RedAutoLeft extends BaseOpMode {
    public static final double TIME_GAP_FOR_LIFT = 0.7;
    public static final double TIME_FACT = 2;
    public static final boolean GO_FOR_3rd_SAMPLE = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-24, -63, Math.PI / 2);
        Pose2d basetPose = new Pose2d(-52, -52, Math.toRadians(45));
        Pose2d sample1Pose = new Pose2d(-49, -48, Math.toRadians(90));
        Pose2d sample2Pose = new Pose2d(-57, -48, Math.toRadians(90));
        Pose2d sample3Pose = new Pose2d(-57, -48, Math.toRadians(115));
        Pose2d parking = new Pose2d(-24, -12, Math.toRadians(0));
        this.initialize(beginPose);

        TrajectoryActionBuilder gotoBasket_1 = drive.actionBuilder(beginPose)
                .splineToLinearHeading(basetPose, Math.toRadians(180));
        TrajectoryActionBuilder gotoSample_1 = gotoBasket_1.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .strafeTo(sample1Pose.position);
        TrajectoryActionBuilder gotoBasket_2 = gotoSample_1.endTrajectory().fresh()
                .splineToLinearHeading(basetPose, Math.toRadians(270));
        TrajectoryActionBuilder gotoSample_2 = gotoBasket_2.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .strafeTo(sample2Pose.position);
        TrajectoryActionBuilder gotoBasket_3 = gotoSample_2.endTrajectory().fresh()
                .splineToLinearHeading(basetPose, Math.toRadians(270));
        TrajectoryActionBuilder gotoSample_3 = gotoBasket_3.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .strafeTo(sample3Pose.position)
                .turnTo(sample3Pose.heading);
        TrajectoryActionBuilder gotoBasket_4 = gotoSample_3.endTrajectory().fresh()
                .splineToLinearHeading(basetPose, Math.toRadians(270));

        TrajectoryActionBuilder gotoParkingAfterSample2 = gotoBasket_3.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .splineToLinearHeading(parking, Math.toRadians(0));
        TrajectoryActionBuilder gotoParkingAfterSample3 = gotoBasket_4.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .splineToLinearHeading(parking, Math.toRadians(0));

        waitForStart();
        // Goto Basket and Drop the Sample
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                this.robot.getLift().liftUpToBasketLevelAction(),
                                new SequentialAction(
                                        new SleepAction(TIME_GAP_FOR_LIFT),
                                        gotoBasket_1.build()
                                )
                        ),
                        this.robot.getLift().openDropClawAction(),
                        new ParallelAction(
                                gotoSample_1.build(),
                                new SequentialAction(
                                        new SleepAction(TIME_GAP_FOR_LIFT),
                                        this.robot.getLift().liftDownAction()
                                )
//                                this.robot.getIntake().OpenClawAction()
                        )
                )
        );
        // Search For Sample-1
        Pose2d samplePose = this.robot.getIntake().searchForSampleUsingSlide(10, 10);
        if (samplePose != null) {
            alignToSample(samplePose);
        }
        // Catch Sample-1 and Drop it into basket
        Actions.runBlocking(
                new SequentialAction(
                        this.robot.getIntake().catchSampleAction(),
                        this.robot.getIntake().goBackAction(),
                        new ParallelAction(
                                this.robot.getLift().liftUpToBasketLevelAction(),
                                new SequentialAction(
                                        new SleepAction(TIME_GAP_FOR_LIFT * TIME_FACT),
                                        gotoBasket_2.build()
                                )
                        ),

                        this.robot.getLift().openDropClawAction(),
                        new ParallelAction(
                                gotoSample_2.build(),
                                new SequentialAction(
                                        new SleepAction(TIME_GAP_FOR_LIFT),
                                        this.robot.getLift().liftDownAction()
                                )
                        )
                )
        );
        // Search For Sample-2
        samplePose = this.robot.getIntake().searchForSampleUsingSlide(10, 10);
        if (samplePose != null) {
            alignToSample(samplePose);
        }
        // Catch Sample-2 and Drop it into basket
        Actions.runBlocking(
                new SequentialAction(
                        this.robot.getIntake().catchSampleAction(),
                        this.robot.getIntake().goBackAction(),
                        new ParallelAction(
                                this.robot.getLift().liftUpToBasketLevelAction(),
                                new SequentialAction(
                                        new SleepAction(TIME_GAP_FOR_LIFT * TIME_FACT),
                                        gotoBasket_3.build()
                                )
                        ),

                        this.robot.getLift().openDropClawAction()
                )
        );

        if (GO_FOR_3rd_SAMPLE) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    gotoSample_3.build(),
                                    new SequentialAction(
                                            new SleepAction(TIME_GAP_FOR_LIFT),
                                            this.robot.getLift().liftDownAction()
                                    )
                            )
                    )
            );

            // Search For Sample-3
            samplePose = this.robot.getIntake().searchForSampleUsingSlide(10, 13);
            if (samplePose != null) {
                alignToSample(samplePose);
            }
            // Catch Sample-2 and Drop it into basket
            Actions.runBlocking(
                    new SequentialAction(
                            this.robot.getIntake().catchSampleAction(),
                            this.robot.getIntake().goBackAction(),

                            new ParallelAction(
                                    this.robot.getLift().liftUpToBasketLevelAction(),
                                    new SequentialAction(
                                            new SleepAction(TIME_GAP_FOR_LIFT * TIME_FACT),
                                            gotoBasket_4.build()
                                    )
                            ),
                            this.robot.getLift().openDropClawAction(),
                            new ParallelAction(
                                    gotoParkingAfterSample3.build(),
                                    new SequentialAction(
                                            new SleepAction(TIME_GAP_FOR_LIFT),
                                            this.robot.getLift().liftDownAction()
                                    )
                            )
                    )
            );
        } else {
            Actions.runBlocking(
                    new ParallelAction(
                            gotoParkingAfterSample2.build(),
                            new SequentialAction(
                                    new SleepAction(TIME_GAP_FOR_LIFT),
                                    this.robot.getLift().liftDownAction()
                            )
                    )
            );
        }
    }

    private void alignToSample(Pose2d samplePose) {
        double angle = samplePose.heading.toDouble();
        if (Math.abs(angle) > 2) {
            Actions.runBlocking(drive.actionBuilder(
                    drive.getPose()).turn(samplePose.heading.toDouble()).build());
        }
    }
}