package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;

/*
 * LIMELIGHT DISTANCE TELEOP
 *
 * CONTROLS (gamepad 1):
 *   Left stick  = drive / strafe
 *   Right stick = turn
 *   X           = toggle pipeline between 1 (RED) and 2 (BLUE)
 *
 * HOW THE DISTANCE IS FOUND:
 *   The Limelight reports ty, the vertical angle from its crosshair to the
 *   target. With the camera's height and mount angle known, trigonometry gives
 *   the straight-line ground distance:
 *
 *       distance = (targetHeight - cameraHeight) / tan(mountAngle + ty)
 *
 *   Fill in the three measurements below for YOUR robot before using the number.
 */
@Utility(name = "Limelight Distance")
public class LimelightDistanceTeleOp extends OpMode {

    // ---- Measure these on your robot (inches / degrees) ----
    private static final double CAMERA_HEIGHT_IN = 10.0;   // Lens center, off the floor.
    private static final double TARGET_HEIGHT_IN = 28.0;   // Center of the target, off the floor.
    private static final double CAMERA_MOUNT_ANGLE_DEG = 20.0; // Tilt up from horizontal (positive = pointing up).

    private static final int PIPELINE_RED = 1;
    private static final int PIPELINE_BLUE = 2;

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

        // Driving (same mapping as MechanismsTeleOp).
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        drivetrain.drive(forward, strafe, turn);

        // Distance from the Limelight.
        LLResult result = limelight.getLatestResult();
        // Ignore results still coming from the previous pipeline right after a toggle.
        boolean usable = result != null
                && result.isValid()
                && result.getPipelineIndex() == currentPipeline;

        telemetry.addData("Pipeline", pipelineName(currentPipeline));
        if (usable) {
            double ty = result.getTy();
            double distanceIn = distanceInches(ty);

            telemetry.addData("Target", "FOUND");
            telemetry.addData("tx (deg)", "%.2f", result.getTx());
            telemetry.addData("ty (deg)", "%.2f", ty);
            telemetry.addData("Distance (in)", "%.1f", distanceIn);
            telemetry.addData("Distance (ft)", "%.2f", distanceIn / 12.0);
        } else {
            telemetry.addData("Target", "NONE");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stopNow();
        limelight.pipelineSwitch(PIPELINE_RED);
        limelight.stop();
    }

    /** Ground distance (inches) to the target center, from the vertical angle ty. */
    private static double distanceInches(double tyDeg) {
        double angleToTargetRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + tyDeg);
        double tan = Math.tan(angleToTargetRad);
        if (Math.abs(tan) < 1e-6) return Double.NaN; // Looking straight at the horizon: no valid distance.
        return (TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN) / tan;
    }

    private static String pipelineName(int pipeline) {
        return pipeline == PIPELINE_RED ? "1 - RED" : "2 - BLUE";
    }
}
