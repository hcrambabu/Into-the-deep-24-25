package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.anime.Intake.INTAKE_SERVO_BACK_POS;
import static org.firstinspires.ftc.teamcode.anime.Lift.LOWER_LIFT_SPECIMEN_DROP_POS;
import static org.firstinspires.ftc.teamcode.anime.Lift.LOWER_LIFT_SPECIMEN_PICKUP_POS;
import static org.firstinspires.ftc.teamcode.anime.Lift.UPPER_LIFT_SPECIMEN_DROP_POS;
import static org.firstinspires.ftc.teamcode.anime.Lift.UPPER_LIFT_SPECIMEN_PICKUP_POS;

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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.anime.BaseOpMode;

@Autonomous(group = "Anime", name = "Right")
//@TeleOp(group = "Anime", name = "Right")
public class RedAutoRight extends BaseOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(24, -64.5, Math.PI/2);
        this.initialize(beginPose);

        VelConstraint pushVelConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 10;
            }
        };

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS, UPPER_LIFT_SPECIMEN_DROP_POS),
                        new ParallelAction(
                                this.robot.getIntake().setIntakeServoPosAction(INTAKE_SERVO_BACK_POS, -0.5, 5),
                                drive.actionBuilder(beginPose)
                                        .splineTo(new Vector2d(-5, -48), Math.toRadians(90))
                                        .strafeTo(new Vector2d(-5, -36))
                                        .build()
                        ),
                        this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_PICKUP_POS, UPPER_LIFT_SPECIMEN_PICKUP_POS)
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.getPose())
                                .strafeTo(new Vector2d(-5, -38))
                                .splineToLinearHeading(new Pose2d(64.5, -48, Math.toRadians(-90)), Math.toRadians(0))
                                .build()
                )
        );

        int tries = 0;
        while (tries < 3 && this.robot.getLift().getSpecimenColorSensor().getDistance(DistanceUnit.CM) > 3) {
            tries++;
            Actions.runBlocking(
                    new SequentialAction(
                            this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_PICKUP_POS, UPPER_LIFT_SPECIMEN_PICKUP_POS),
                            drive.actionBuilder(drive.getPose())
                                    .strafeTo(new Vector2d(64.5, -60.5))
                                    .build(),
                            new ParallelAction(
                                    this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS, UPPER_LIFT_SPECIMEN_DROP_POS),
                                    new SequentialAction(
                                            new SleepAction(.2),
                                            drive.actionBuilder(drive.getPose())
                                                    .strafeTo(new Vector2d(64.5, -48))
                                                    .build()
                                    )

                            )
                    )
            );
        }

        if(this.robot.getLift().getSpecimenColorSensor().getDistance(DistanceUnit.CM) < 2.5) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    this.robot.getIntake().setIntakeServoPosAction(INTAKE_SERVO_BACK_POS, -0.5, 5),
                                    drive.actionBuilder(beginPose)
                                            .splineToLinearHeading(new Pose2d(2, -52, Math.toRadians(90)), Math.toRadians(180))
                                            .build()
                            )
                    )
            );
        }

        tries = 0;
        while (tries < 3 && this.robot.getLift().getSpecimenColorSensor().getDistance(DistanceUnit.CM) < 2.5) {
            tries++;
            Actions.runBlocking(
                    new SequentialAction(
                            this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS, UPPER_LIFT_SPECIMEN_DROP_POS+100),
                            drive.actionBuilder(beginPose)
                                    .strafeTo(new Vector2d(2, -35))
                                    .build(),
                            this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_PICKUP_POS, UPPER_LIFT_SPECIMEN_PICKUP_POS),
                            drive.actionBuilder(beginPose)
                                    .strafeTo(new Vector2d(2, -48))
                                    .build()
                    )
            );
        }

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(beginPose)
                                .splineToLinearHeading(new Pose2d(64.5, -64.5, Math.toRadians(90)), Math.toRadians(0))
                                .build()
                )
        );


//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                drive.actionBuilder(drive.getPose())
//                                        .strafeTo(new Vector2d(0, -48))
//                                        .splineToLinearHeading(new Pose2d(24, -48, Math.toRadians(-90)), Math.toRadians(90))
//                                        .splineToLinearHeading(new Pose2d( 50, -12, Math.toRadians(-90)),Math.toRadians(-90))
//                                        .strafeTo(new Vector2d(50, -12))
//                                        .strafeTo(new Vector2d(50, -56))
//                                        .strafeTo(new Vector2d(50, 0))
//                                        .strafeTo(new Vector2d(64.5, 0))
//                                        .strafeTo(new Vector2d(64.5, -52))
//                                        .strafeTo(new Vector2d(64.5, -64.5))
//                                        .build(),
//                                this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_PICKUP_POS, UPPER_LIFT_SPECIMEN_PICKUP_POS)
//                        )
//                )
//        );
    }
}