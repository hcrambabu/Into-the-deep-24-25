package org.firstinspires.ftc.teamcode.Auto;

//RR-specific imports

// Non-RR imports
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Drive;
@Config
@Autonomous(name="redAuto", group="Robot")
// Don't edit!!!!!!!!!!!!!!!!!!!!!!
public abstract class redAuto extends LinearOpMode {

    public void runOpMode() {


        if(opModeIsActive()){

            Drive.driveTiles(1);
            Drive.setRotateDegrees(45);

        }




}}

