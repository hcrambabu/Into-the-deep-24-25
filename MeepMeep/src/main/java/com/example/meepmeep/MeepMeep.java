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

    public static void main(String[] args) {

        VelConstraint pushVelConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(Pose2dDual<Arclength> pose2dDual, PosePath posePath, double v) {
                return 10;
            }
        };

        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 9.5)
                .setDimensions(15, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, -64.5, Math.PI/2))
                .splineTo(new Vector2d(0, -36), Math.toRadians(90))
                .splineTo(new Vector2d(36, -36), Math.toRadians(90))
                //.strafeTo(new Vector2d(24, -36))
                .splineToLinearHeading(new Pose2d( 50, -12, Math.toRadians(-90)),Math.toRadians(-90))
                .strafeTo(new Vector2d(50, -12))
                .strafeTo(new Vector2d(50, -56))
                .strafeTo(new Vector2d(50, -12))
                .strafeTo(new Vector2d(64.5, -12))
                .strafeTo(new Vector2d(64.5, -52))
                .strafeTo(new Vector2d(64.5, -12))
                .build());

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}