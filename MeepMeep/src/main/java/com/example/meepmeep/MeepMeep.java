package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeep {
    public static final boolean GO_FOR_3rd_SAMPLE = false;
    public static final boolean IS_LEFT = false;

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, -12, Math.toRadians(0)))
//                .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(180))
//                    .build());
//        };


        if(IS_LEFT) {
            // Left
            Pose2d beginPose = new Pose2d(-24, -63, Math.PI / 2);
            Pose2d basetPose = new Pose2d(-52, -52, Math.toRadians(45));
            Pose2d sample1Pose = new Pose2d(-49, -48, Math.toRadians(90));
            Pose2d sample2Pose = new Pose2d(-57, -48, Math.toRadians(90));
            Pose2d sample3Pose = new Pose2d(-57, -48, Math.toRadians(115));
            Pose2d parking = new Pose2d(-24, -12, Math.toRadians(0));

            TrajectoryActionBuilder gotoBasket_1 = myBot.getDrive().actionBuilder(beginPose)
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

            if (GO_FOR_3rd_SAMPLE) {
                myBot.runAction(
                        new SequentialAction(
                                gotoBasket_1.build(),
                                gotoSample_1.build(),
                                gotoBasket_2.build(),
                                gotoSample_2.build(),
                                gotoBasket_3.build(),
                                gotoSample_3.build(),
                                gotoBasket_4.build(),
                                gotoParkingAfterSample3.build()

                        ));
            } else {
                myBot.runAction(
                        new SequentialAction(
                                gotoBasket_1.build(),
                                gotoSample_1.build(),
                                gotoBasket_2.build(),
                                gotoSample_2.build(),
                                gotoBasket_3.build(),
                                gotoParkingAfterSample2.build()
                        ));
            }
        } else {
            // Right
            VelConstraint pushVelConstraint = new VelConstraint() {
                @Override
                public double maxRobotVel(Pose2dDual<Arclength> pose2dDual, PosePath posePath, double v) {
                    return 10;
                }
            };
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -12, Math.toRadians(0)))
                            .setTangent(135)
                    .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(180))
                    .build());
        }

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}