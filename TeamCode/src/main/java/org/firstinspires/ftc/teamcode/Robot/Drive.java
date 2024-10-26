package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auto.redAuto;

import java.util.concurrent.TimeUnit;

public abstract class Drive extends LinearOpMode {
    static HardwareMap hMap = null;
    static DcMotor DcMotorA = null;
    static DcMotor DcMotorB = null;
    static DcMotor DcMotorC = null;
    static DcMotor DcMotorD = null;

    public static void init(HardwareMap hardwareMap){
        hMap = hardwareMap;

        DcMotorA = hMap.get(DcMotor.class, "Drive1");
        DcMotorB = hMap.get(DcMotor.class, "Drive2");
        DcMotorC = hMap.get(DcMotor.class, "Drive3");
        DcMotorD = hMap.get(DcMotor.class, "Drive4");
        DcMotorC.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

   public static void driveTiles(int Tiles) {
       DcMotorA.setTargetPosition(-993 * Tiles);
       DcMotorB.setTargetPosition(-1044 * Tiles);
       DcMotorC.setTargetPosition(-1037 * Tiles);
       DcMotorD.setTargetPosition(-1061 * Tiles);
       DcMotorA.setPower(1);
       DcMotorB.setPower(1);
       DcMotorC.setPower(1);
       DcMotorD.setPower(1);
       DcMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       DcMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       DcMotorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       DcMotorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       DcMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

   public static void setRotateDegrees(int Deg) {
       DcMotorA.setTargetPosition(23 * Deg);
       DcMotorB.setTargetPosition(18 *Deg);
       DcMotorC.setTargetPosition(-20 *Deg);
       DcMotorD.setTargetPosition(-20 *Deg);
       DcMotorA.setPower(1);
       DcMotorB.setPower(1);
       DcMotorC.setPower(1);
       DcMotorD.setPower(1);
       DcMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       DcMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       DcMotorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       DcMotorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    }

