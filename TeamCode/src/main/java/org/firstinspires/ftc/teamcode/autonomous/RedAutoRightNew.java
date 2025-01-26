package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.anime.Intake.INTAKE_SERVO_BACK_POS;
import static org.firstinspires.ftc.teamcode.anime.Lift.LOWER_LIFT_SPECIMEN_DROP_POS;
import static org.firstinspires.ftc.teamcode.anime.Lift.LOWER_LIFT_SPECIMEN_DROP_POS_2;
import static org.firstinspires.ftc.teamcode.anime.Lift.LOWER_LIFT_SPECIMEN_PICKUP_POS;
import static org.firstinspires.ftc.teamcode.anime.Lift.UPPER_LIFT_SPECIMEN_DROP_POS;
import static org.firstinspires.ftc.teamcode.anime.Lift.UPPER_LIFT_SPECIMEN_DROP_POS_2;
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
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.anime.BaseOpMode;

@Autonomous(group = "Anime", name = "RightNew")
//@TeleOp(group = "Anime", name = "RightNew")
public class RedAutoRightNew extends BaseOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double dropX = 0;
        double dropY = -35;
        double pickupX = 46;
        double pickupY = -60.5;
        Pose2d beginPose = new Pose2d(0, -64.5, Math.PI/2);
        this.initialize(beginPose, false);

        VelConstraint pushVelConstraint = new VelConstraint() {
            @Override
            public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                return 40;
            }
        };

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS, UPPER_LIFT_SPECIMEN_DROP_POS),
                                this.robot.getIntake().setIntakeServoPosAction(INTAKE_SERVO_BACK_POS, -0.5, 5),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        drive.actionBuilder(beginPose)
                                                .strafeTo(new Vector2d(dropX, dropY))
                                                .build()
                                )
                        ),
                        this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS_2, UPPER_LIFT_SPECIMEN_DROP_POS_2)
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(drive.getPose())
                                .strafeTo(new Vector2d(dropX, -48))
                                .splineToLinearHeading(new Pose2d(29, -48, Math.toRadians(-90)), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d( pickupX, -12, Math.toRadians(-90)),Math.toRadians(-90))
                                .strafeTo(new Vector2d(pickupX, -12))
                                .strafeTo(new Vector2d(pickupX, pickupY), pushVelConstraint)
                                .build(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_PICKUP_POS, UPPER_LIFT_SPECIMEN_PICKUP_POS)
                        )
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS, UPPER_LIFT_SPECIMEN_DROP_POS),
                                new SequentialAction(
                                        new SleepAction(.2),
                                        drive.actionBuilder(drive.getPose())
                                                .strafeTo(new Vector2d(pickupX, -48))
                                                .splineToConstantHeading(new Vector2d( 60, 0),Math.toRadians(0))
                                                .strafeTo(new Vector2d(60, pickupY+8))
                                                .strafeTo(new Vector2d(pickupX, -48))
                                                .build()
                                )
                        )
                )
        );

        for(int i = 0; i < 3; i++) {
            int tries = 0;
            while (tries < 3 && this.robot.getLift().getSpecimenColorSensor().getDistance(DistanceUnit.CM) > 3) {
                tries++;
                Actions.runBlocking(
                        new SequentialAction(
                                this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_PICKUP_POS, UPPER_LIFT_SPECIMEN_PICKUP_POS),
                                drive.actionBuilder(drive.getPose())
                                        .strafeTo(new Vector2d(pickupX, pickupY))
                                        .build()
                        )
                );
                Actions.runBlocking(
                        new ParallelAction(
                                this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS, UPPER_LIFT_SPECIMEN_DROP_POS),
                                new SequentialAction(
                                        new SleepAction(.2),
                                        drive.actionBuilder(drive.getPose())
                                                .strafeTo(new Vector2d(pickupX, -48))
                                                .build()
                                )
                        )
                );
            }

            dropX += 2;
            pickupX = 48;
            if (this.robot.getLift().getSpecimenColorSensor().getDistance(DistanceUnit.CM) < 2.5) {
                Actions.runBlocking(
                        new SequentialAction(
                                new ParallelAction(
                                        this.robot.getIntake().setIntakeServoPosAction(INTAKE_SERVO_BACK_POS, -0.5, 5),
                                        drive.actionBuilder(drive.getPose())
                                                .splineToLinearHeading(new Pose2d(dropX, -48, Math.toRadians(90)), Math.toRadians(180))
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
                                this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS, UPPER_LIFT_SPECIMEN_DROP_POS),
                                drive.actionBuilder(drive.getPose())
                                        .strafeTo(new Vector2d(dropX,  dropY))
                                        .build(),
                                this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS_2, UPPER_LIFT_SPECIMEN_DROP_POS_2)
                        )
                );
                Actions.runBlocking(
                        drive.actionBuilder(drive.getPose())
                                .strafeTo(new Vector2d(dropX, -48))
                                .build()
                );
            }

            if(i == 2) {
                break;
            }

            Actions.runBlocking(
                    new ParallelAction(
                            this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_PICKUP_POS, UPPER_LIFT_SPECIMEN_PICKUP_POS),
                            drive.actionBuilder(drive.getPose())
                                    .splineToLinearHeading(new Pose2d(pickupX, -48, Math.toRadians(-90)), Math.toRadians(0))
                                    .strafeTo(new Vector2d(pickupX, pickupY))
                                    .build()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            this.robot.getLift().liftAction(LOWER_LIFT_SPECIMEN_DROP_POS, UPPER_LIFT_SPECIMEN_DROP_POS),
                            new SequentialAction(
                                    new SleepAction(.2),
                                    drive.actionBuilder(drive.getPose())
                                            .strafeTo(new Vector2d(pickupX, -48))
                                            .build()
                            )
                    )
            );
        }

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.getPose())
                                .splineToLinearHeading(new Pose2d(58, -58, Math.toRadians(-90)), Math.toRadians(0))
                                .build()
                )
        );

        while (opModeIsActive()) {
            Thread.yield();
        }
    }
}