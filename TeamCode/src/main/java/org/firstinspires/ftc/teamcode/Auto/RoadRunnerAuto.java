package org.firstinspires.ftc.teamcode.Auto;

//RR-specific imports

// Non-RR imports
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Drive;
@Config
@Autonomous(name="RoadRunnerAuto", group="Robot")
// Don't edit!!!!!!!!!!!!!!!!!!!!!!
public abstract class RoadRunnerAuto extends LinearOpMode {
    // instantiate your MecanumDrive at a particular pose.
    Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    Action trajectoryAction1 = drive.actionBuilder(drive.pose)
            .splineTo(new Vector2d(46.96, -5.61), Math.toRadians(254.48))
            .splineTo(new Vector2d(46.08, -60.06), Math.toRadians(258.69))
            .splineTo(new Vector2d(48.56, 3.57), Math.toRadians(-61.04))
            .splineTo(new Vector2d(58.75, -63.85), Math.toRadians(-81.40))
            .splineTo(new Vector2d(46.81, 4.73), Math.toRadians(90.00))
            .splineTo(new Vector2d(65.16, -6.92), Math.toRadians(-83.09))
            .splineTo(new Vector2d(68.36, -59.33), Math.toRadians(-86.50))
            .splineTo(new Vector2d(-24.10, -46.96), Math.toRadians(172.38))
            .splineTo(new Vector2d(-31.96, -7.50), Math.toRadians(170.84))
            .splineTo(new Vector2d(-45.79, -24.53), Math.toRadians(270.00))
            .splineTo(new Vector2d(55.98, -64.57), Math.toRadians(-21.48))
            .splineTo(new Vector2d(50.01, -67.78), Math.toRadians(208.22))
            .splineTo(new Vector2d(-57.88, -11.14), Math.toRadians(264.43))
            .splineTo(new Vector2d(0.80, -63.70), Math.toRadians(-3.03))
            .splineTo(new Vector2d(53.65, -64.43), Math.toRadians(-0.79))
            .splineTo(new Vector2d(-64.57, 0.22), Math.toRadians(117.93))
            .splineTo(new Vector2d(-66.90, -15.94), Math.toRadians(262.28))
            .splineTo(new Vector2d(45.65, -66.32), Math.toRadians(0.00))
            .build();

    @Override
    public void runOpMode() {
        waitForStart();
        if (isStopRequested()) return;
        }
    }

