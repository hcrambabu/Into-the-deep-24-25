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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ForkJoinPool;
import java.util.logging.Logger;


public class AnimeRobot {

    public static final double SLOW_RUN_MULTIPLIER = 0.4;

    public static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
    public static final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    private static Logger log = Logger.getLogger(AnimeRobot.class.getName());

    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
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
        this.setupDriveMotors();
        this.drive = new PinpointDrive(this.hardwareMap, beginPose);
        this.lift = new Lift(this);
        this.intake = new Intake(this);
    }


    public void setupDriveMotors() {
        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        this.backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void  handleKeyPress(Gamepad gamepad1, Gamepad gamepad2) {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        if (gamepad1.right_trigger > 0.5) {
            y *= SLOW_RUN_MULTIPLIER;
            x *= SLOW_RUN_MULTIPLIER;
            rx *= SLOW_RUN_MULTIPLIER;
        }

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        this.getOpMode().telemetry.addData("Wheel Powers", String.format("LF: %.2f, LB: %.2f, RF: %.2f, RB: %.2f",
                frontLeftMotor.getPower(),
                backLeftMotor.getPower(),
                frontRightMotor.getPower(),
                backRightMotor.getPower()
        ));
        this.getOpMode().telemetry.addData("Wheel Velocity", String.format("LF: %.2f, LB: %.2f, RF: %.2f, RB: %.2f",
                frontLeftMotor.getVelocity(AngleUnit.DEGREES),
                backLeftMotor.getVelocity(AngleUnit.DEGREES),
                frontRightMotor.getVelocity(AngleUnit.DEGREES),
                backRightMotor.getVelocity(AngleUnit.DEGREES)
        ));
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
