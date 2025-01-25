package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.anime.Intake.INTAKE_SERVO_BACK_POS;
import static org.firstinspires.ftc.teamcode.anime.Intake.INTAKE_SERVO_START_POS;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.anime.BaseOpMode;
import org.firstinspires.ftc.teamcode.anime.ColorUtility;

@Autonomous(group = "Anime", name = "Left")
//@TeleOp(group = "Anime", name = "Left")
public class RedAutoLeft extends BaseOpMode {
    public static final double TIME_GAP_FOR_LIFT = 0.2;
    public static final double TIME_FACT = 2;
    public static final boolean GO_FOR_3rd_SAMPLE = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-24, -64.5, Math.PI / 2);
        Pose2d basketPose = new Pose2d(-58, -62, Math.toRadians(45));
        Pose2d parking = new Pose2d(-24, -12, Math.toRadians(0));
        this.initialize(beginPose);

        TrajectoryActionBuilder gotoBasket_1 = drive.actionBuilder(beginPose)
                .splineToLinearHeading(basketPose, Math.toRadians(180));

        waitForStart();
        // Goto Basket and Drop the Sample
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                this.robot.getIntake().startIntakeCRServoAction(),
                                gotoBasket_1.build(),
                                this.robot.getLift().liftUpAction(),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        this.robot.getIntake().setIntakeServoPosAction(INTAKE_SERVO_BACK_POS, -0.5, 5)
                                )
                        ),
                        this.robot.getIntake().releaseSampleAction(),
                        new SleepAction(TIME_GAP_FOR_LIFT)
                )
        );
        // Search For Samples
        Pose2d[] samplePoses = {
                new Pose2d(-30, -36, Math.toRadians(172)),
                new Pose2d(-40, -36, Math.toRadians(172)),
                new Pose2d(-50, -36, Math.toRadians(172))
        };
        for (Pose2d samplePose : samplePoses) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    drive.actionBuilder(this.robot.getDrive().getPose())
                                            .splineToLinearHeading(samplePose, Math.toRadians(180))
                                            .turn(Math.toRadians(170))
                                            .build()
                            ),
                            new ParallelAction(
                                    this.robot.getLift().liftAction(200, 2000)
                            ),
                            this.robot.getIntake().setIntakeServoPosAction(60, -0.25, 5),
                            this.robot.getIntake().startIntakeCRServoAction()
                    )
            );

            boolean found = searchForSampleUsingDistance(5, new PoseVelocity2d(new Vector2d(0.30, 0), 0));
            if (found) {
                Actions.runBlocking(
                        new SequentialAction(
                                new ParallelAction(
                                        drive.actionBuilder(this.robot.getDrive().getPose())
                                                .splineToLinearHeading(basketPose, Math.toRadians(180)).build(),
                                        this.robot.getLift().liftUpAction(),
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                this.robot.getIntake().setIntakeServoPosAction(INTAKE_SERVO_BACK_POS, -0.5, 5)
                                        )
                                ),
                                this.robot.getIntake().releaseSampleAction(),
                                new SleepAction(TIME_GAP_FOR_LIFT)
                        )
                );
            }
        }

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(this.robot.getDrive().getPose())
                                .splineToLinearHeading(parking, Math.toRadians(0)).build(),
                        new SequentialAction(
                                new SleepAction(TIME_GAP_FOR_LIFT),
                                this.robot.getLift().liftHomeAction()
                        )
                )
        );

        while(opModeIsActive()) {
            Pose2d pose = this.robot.getDrive().getPose();
            telemetry.addData("Pose", String.format("X: %.2f, Y: %.2f, H: %.2f", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble())));
            this.robot.getIntake().updateTelemetry();
            this.robot.getLift().updateTelemetry();
            telemetry.update();
            Thread.yield();
        }
    }

    private boolean searchForSampleUsingDistance(double timeoutSec, PoseVelocity2d velocity) {
        long timeout = System.currentTimeMillis() + (long) (timeoutSec * 1000);
        double intakeSensorDistance = this.robot.getIntake().getIntakeColorSensor().getDistance(DistanceUnit.CM);
        while (System.currentTimeMillis() < timeout && intakeSensorDistance > 1) {
            this.robot.getDrive().setDrivePowers(velocity);
            Thread.yield();
            intakeSensorDistance = this.robot.getIntake().getIntakeColorSensor().getDistance(DistanceUnit.CM);
        }
        this.robot.getDrive().setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        return intakeSensorDistance < 1;
    }

    private boolean searchForSampleUsingColor(ColorUtility.Color color, double timeoutSec, PoseVelocity2d velocity) {
        long timeout = System.currentTimeMillis() + (long) (timeoutSec * 1000);
        ColorUtility.Color sampleColor = ColorUtility.getColorName(this.robot.getIntake().getIntakeColorSensor().getNormalizedColors());
        while (System.currentTimeMillis() < timeout && sampleColor != color) {
            this.robot.getDrive().setDrivePowers(velocity);
            Thread.yield();
            sampleColor = ColorUtility.getColorName(this.robot.getIntake().getIntakeColorSensor().getNormalizedColors());
        }
        this.robot.getDrive().setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        return sampleColor == color;
    }

    private boolean searchForSampleUsingColor(ColorUtility.Color color, double timeoutSec) {
        double fVel = 0.25;
        double sVel = 0.25;
        double aVel = 0.25;
        PoseVelocity2d[] velocity2ds = {
                new PoseVelocity2d(new Vector2d(fVel, 0), 0),
                new PoseVelocity2d(new Vector2d(-fVel, 0), 0),
                new PoseVelocity2d(new Vector2d(0, sVel), 0),
                new PoseVelocity2d(new Vector2d(0, -sVel), 0),
                new PoseVelocity2d(new Vector2d(0, 0), aVel),
                new PoseVelocity2d(new Vector2d(0, 0), -aVel),
        };

        for (PoseVelocity2d velocity2d : velocity2ds) {
            if (searchForSampleUsingColor(color, timeoutSec, velocity2d)) {
                return true;
            }
        }
        return false;
    }
}