package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, -63, Math.toRadians(90)))
////                        .setTangent(0)
//                                .splineTo(new Vector2d(48, -24), Math.toRadians(90))
//                .lineToY(24)
////                .turn(Math.toRadians(90))
////                .lineToY(30)
////                .turn(Math.toRadians(90))
////                .lineToX(0)
////                .turn(Math.toRadians(90))
////                .lineToY(0)
////                .turn(Math.toRadians(90))
//                .build());

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -63, Math.toRadians(90)))
//                        .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(45))
                .setTangent(0)
                        .splineTo(new Vector2d(-60, -60), Math.toRadians(45))
                        .build());


        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}