package org.firstinspires.ftc.teamcode.anime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ForkJoinPool;
import java.util.logging.Logger;


public class AnimeRobot {

    public static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
    public static final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    private static Logger log = Logger.getLogger(AnimeRobot.class.getName());
    IMU imu;
    HardwareMap hardwareMap;
    private LinearOpMode opMode;
    private ElapsedTime robotRuntime = new ElapsedTime();

    private MecanumDrive drive;
    private Intake intake;
    private Lift lift;

    public AnimeRobot(LinearOpMode opMode, Pose2d beginPose) {
        log.info("Initializing AnimeRobot, pool size:" + ForkJoinPool.getCommonPoolParallelism());
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        this.imu = hardwareMap.get(IMU.class, "imu");

        this.drive = new SparkFunOTOSDrive(this.hardwareMap, beginPose);
        this.lift = new Lift(this);
        this.intake = new Intake(this);
    }
    public IMU getImu() {
        return imu;
    }
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
    public ElapsedTime getRobotRuntime() {
        return robotRuntime;
    }
    public LinearOpMode getOpMode() {
        return opMode;
    }
    public MecanumDrive getDrive() {
        return drive;
    }
    public void sleep(long millis) {
        this.opMode.sleep(millis);
    }

    public Intake getIntake() {
        return intake;
    }

    public Lift getLift() {
        return lift;
    }

    private CompletableFuture<Void> currentGoBackTask;
    public void goToBasket() {
        if (currentGoBackTask != null && !currentGoBackTask.isDone() && !currentGoBackTask.isCancelled()) {
            return;
        }

        currentGoBackTask = CompletableFuture.runAsync(() -> {
            Actions.runBlocking(
                    drive.actionBuilder(drive.getPose())
                            .setTangent(135)
                            .splineToLinearHeading(new Pose2d(-52, -52, Math.toRadians(45)), Math.toRadians(180))
                            .build()
            );
        });
    }
}
