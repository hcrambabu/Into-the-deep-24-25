package org.firstinspires.ftc.teamcode.anime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;

public class IntakeHusky {

    public static final float SCREEN_WIDTH = 320; //pixels
    public static final float SCREEN_HEIGHT = 240; // pixels
    public static final float CLAW_POS_X_ON_SCREEN_WRT_CAMERA_START = 230; //pixels
    public static final float CLAW_POS_Y_ON_SCREEN_WRT_CAMERA_START = 100; // pixels
    public static final Vector2d CLAW_POS_WRT_ROBOT_CENTER = new Vector2d(0, 12.5); // inches

    private AnimeRobot robot;
    private HuskyLens huskyLens;

    private PerspectiveTransformer perspectiveTransformer;
    private Vector2d ClawPosOnMatWrtCameraStart;

    public IntakeHusky(AnimeRobot robot) {
        this.robot = robot;
        this.huskyLens = robot.getIntakeHuskyLens();

        this.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        if (!huskyLens.knock()) {
            robot.getOpMode().telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            robot.getOpMode().telemetry.addData(">>", "Press start to continue");
        }

        perspectiveTransformer = new PerspectiveTransformer(
                new float[]{0.0f, 0.0f, SCREEN_WIDTH, 0.0f, SCREEN_WIDTH, SCREEN_HEIGHT, 0.0f, SCREEN_HEIGHT},
                new float[]{0.0f, 0.0f, 16.0f, 0.0f, 13.5f, 11.5f, 3.0f, 11.5f} // quadrilateral (Camera Visible area) on mat coordinates in inches
        );
        ClawPosOnMatWrtCameraStart = perspectiveTransformer.transformPoint(CLAW_POS_X_ON_SCREEN_WRT_CAMERA_START, CLAW_POS_Y_ON_SCREEN_WRT_CAMERA_START);
    }

    public void selectAlgorithm(HuskyLens.Algorithm algorithm) {
        this.huskyLens.selectAlgorithm(algorithm);
    }

    public HuskyLens.Block[] blocks() {
        return huskyLens.blocks();
    }

    public Pose2d didYouFind() {
        HuskyLens.Block[] blocks = blocks();
        if (blocks.length == 0) {
            return null;
        }

        HuskyLens.Block block = blocks[0];
        this.robot.getOpMode().telemetry.addData("block pos wrt screen", block);
        // Transform the block pos wrt to center of robot
        Vector2d blockPosOnMatWrtCameraStart = perspectiveTransformer.transformPoint((float) block.x, (float)block.y);
        Vector2d blockPosWrtCenterOfRobot = blockPosOnMatWrtCameraStart.minus(ClawPosOnMatWrtCameraStart).plus(CLAW_POS_WRT_ROBOT_CENTER);
        this.robot.getOpMode().telemetry.addData("block pos wrt robot", String.format("X:%.2f, Y:%.2f", blockPosWrtCenterOfRobot.x, blockPosWrtCenterOfRobot.y));

        Rotation2d angleWrtCenterOfRobot = Rotation2d.exp(Math.atan2(blockPosWrtCenterOfRobot.y, blockPosWrtCenterOfRobot.x)).plus(Math.toRadians(-90)).plus(Math.toRadians(2)); // Subtract 90 to make wrt to Y axis and 2 degrees to corrent error
        return new Pose2d(blockPosWrtCenterOfRobot, angleWrtCenterOfRobot);
    }
}
//        // transform the block pos wrt to center of field.
//        Pose2d currentRobotPos = this.robot.getDrive().getPose();
//        public static final Rotation2d ROBOT_PLANE_VS_FIELD_ANGLE = Rotation2d.exp(Math.toRadians(-90));
//        Vector2d rotatedBlockPosWrtFieldCenter = currentRobotPos.heading.times(ROBOT_PLANE_VS_FIELD_ANGLE.times(blockPosWrtCenterOfRobot)).plus(currentRobotPos.position);
//        return rotatedBlockPosWrtFieldCenter;