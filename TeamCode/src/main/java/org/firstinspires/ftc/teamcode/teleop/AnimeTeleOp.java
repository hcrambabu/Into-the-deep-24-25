package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.anime.BaseOpMode;
import org.firstinspires.ftc.teamcode.anime.PoseStorage;

import java.util.logging.Logger;

@TeleOp(group = "Anime", name = "Anime: TeleOp")
public class AnimeTeleOp extends BaseOpMode {

    public static final double SLOW_RUN_MULTIPLIER = 0.2;
    private static Logger log = Logger.getLogger(AnimeTeleOp.class.getName());

    @Override
    public void initialize(Pose2d beginPose) {
        super.initialize(beginPose);
    }

    private void updateTelemetry() {
    }

    public void runLoop() throws InterruptedException {
        handleMecanum();
        shouldGoToBasket();
        this.robot.getLift().handleKeyPress(gamepad1, gamepad2);
        this.robot.getIntake().handleKeyPress(gamepad1, gamepad2);
        updateTelemetry();
    }

    private void shouldGoToBasket() {
        if(gamepad1.right_trigger > 0.5) {
            this.robot.goToBasket();
        }
    }

    private void handleMecanum() {

        double g1ly = -gamepad1.left_stick_y;
        double g1lx = -gamepad1.left_stick_x;
        double g1rx = -gamepad1.right_stick_x;
        if (gamepad1.left_stick_button) {
            g1ly *= SLOW_RUN_MULTIPLIER;
            g1lx *= SLOW_RUN_MULTIPLIER;
            g1rx *= SLOW_RUN_MULTIPLIER;
        }

        this.robot.getDrive().setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        g1ly,
                        g1lx
                ),
                g1rx
        ));

        this.robot.getDrive().updatePoseEstimate();
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.fieldOverlay().setStroke("#3F51B5");
//        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("Before initialize....");
        this.initialize(PoseStorage.currentPose); // TODO get from autonomous pose
        log.info("before Start....");
        telemetry.update();
        waitForStart();
        log.info("After Start....");

        while (opModeIsActive()) {
            runLoop();
            telemetry.update();
        }
    }
}
