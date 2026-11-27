package org.firstinspires.ftc.teamcode.teleopVariants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

import dev.nextftc.robot.opmode.NextTeleop;

/*
 * LIMELIGHT ALIGN TELEOP
 *
 * CONTROLS (gamepad 1):
 *   Left stick  = drive / strafe (always works)
 *   Right stick = turn (only when NOT aligning)
 *   Right trigger (hold) = auto-aim: the robot turns itself until the target
 *                          is centered in the Limelight's view
 *   X           = toggle pipeline between 1 (RED) and 2 (BLUE)
 *
 * HOW THE ALIGN WORKS:
 *   tx is the horizontal angle from the Limelight's crosshair to the target.
 *   While the trigger is held we set turn power = kP * tx, so the robot turns
 *   toward the target and tx shrinks to zero. If no target is seen, we don't turn.
 */
@NextTeleop(name = "Limelight Align")
public class LimelightAlignTeleOp extends OpMode {

    private static final int PIPELINE_RED = 1;
    private static final int PIPELINE_BLUE = 2;

    // ---- Tuning ----
    private static final double ALIGN_TRIGGER_THRESHOLD = 0.5; // Trigger past this = aligning.
    private static final double KP_TURN = 0.02;   // Power per degree of tx. Raise for snappier, lower if it oscillates.
    private static final double MIN_TURN_POWER = 0.08; // Overcomes carpet friction near the center.
    private static final double MAX_TURN_POWER = 0.6;  // Cap so the robot doesn't whip around.
    private static final double ALIGN_DEADBAND_DEG = 1.0; // Within this many degrees counts as aligned.

    private Limelight3A limelight;
    private final Drivetrain drivetrain = new Drivetrain();

    private int currentPipeline = PIPELINE_RED;
    private boolean lastX = false;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(currentPipeline);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Limelight", limelight.isConnected() ? "CONNECTED" : "NOT CONNECTED");
        telemetry.addData("Pipeline", pipelineName(currentPipeline));
        telemetry.update();
    }

    @Override
    public void loop() {
        // X toggles RED <-> BLUE on the press, not every loop it is held.
        boolean x = gamepad1.x;
        if (x && !lastX) {
            currentPipeline = (currentPipeline == PIPELINE_RED) ? PIPELINE_BLUE : PIPELINE_RED;
            limelight.pipelineSwitch(currentPipeline);
        }
        lastX = x;

        // Ignore results still coming from the previous pipeline right after a toggle.
        LLResult result = limelight.getLatestResult();
        boolean targetVisible = result != null
                && result.isValid()
                && result.getPipelineIndex() == currentPipeline;

        boolean aligning = gamepad1.right_trigger > ALIGN_TRIGGER_THRESHOLD;

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        String status;
        double tx = 0;
        if (aligning) {
            if (!targetVisible) {
                turn = 0;
                status = "ALIGNING - NO TARGET";
            } else {
                tx = result.getTx();
                if (Math.abs(tx) <= ALIGN_DEADBAND_DEG) {
                    turn = 0;
                    status = "ALIGNED";
                } else {
                    turn = computeTurnPower(tx);
                    status = "ALIGNING";
                }
            }
        } else {
            status = "DRIVING";
        }

        drivetrain.drive(forward, strafe, turn);

        telemetry.addData("Pipeline", pipelineName(currentPipeline));
        telemetry.addData("Mode", status);
        telemetry.addData("Target", targetVisible ? "FOUND" : "NONE");
        if (targetVisible) {
            telemetry.addData("tx (deg)", "%.2f", result.getTx());
            telemetry.addData("ty (deg)", "%.2f", result.getTy());
        }
        telemetry.addData("Turn power", "%.2f", turn);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stopNow();
        limelight.pipelineSwitch(PIPELINE_RED);
        limelight.stop();
    }

    /**
     * Proportional turn toward the target. Positive tx means the target is right of
     * center, and positive turn is clockwise in Drivetrain.drive(), so the sign carries over directly.
     */
    private static double computeTurnPower(double tx) {
        double power = KP_TURN * tx;
        // Add a small push in the right direction so the robot doesn't stall just short of center.
        power += Math.signum(power) * MIN_TURN_POWER;
        return Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    private static String pipelineName(int pipeline) {
        return pipeline == PIPELINE_RED ? "1 - RED" : "2 - BLUE";
    }
}
