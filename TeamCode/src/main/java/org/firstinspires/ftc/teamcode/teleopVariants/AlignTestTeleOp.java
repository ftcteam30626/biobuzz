package org.firstinspires.ftc.teamcode.teleopVariants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.teleOp.FearOfBot;

import dev.nextftc.robot.opmode.NextOpMode;   // Base class for NextFTC OpModes.
import dev.nextftc.robot.opmode.NextTeleop;   // Puts this OpMode in the Driver Station's TeleOp list.
import dev.nextftc.robot.triggers.CommandGamepad;

/*
 * ALIGN TEST TELEOP (NextFTC)
 *
 * CONTROLS (gamepad 1):
 *   Left stick    = drive / strafe
 *   Right stick   = turn (only when NOT aligning)
 *   Right trigger = HOLD to align: the robot turns until it is square to the AprilTag
 *   X             = shoot
 *   A             = intake + transfer on / off
 *
 * HOW THE ALIGN WORKS:
 *   The Limelight reports the tag's yaw relative to the camera. 0 = facing the tag straight on.
 *   While the trigger is held, the robot turns until the yaw is within ALIGN_DEADBAND_DEG of 0.
 *   If it turns the wrong way, set TURN_SIGN to -1.0.
 *
 * The Limelight must be on its AprilTag pipeline (switch it in the Limelight web UI, or use the
 * Limelight's own pipeline). This code uses whichever pipeline is active.
 */
@NextTeleop(name = "Align Test TeleOp", group = "NextFTC")
public class AlignTestTeleOp extends NextOpMode {

    private static final double TURN_SIGN = 1.0;           // -1.0 if the robot turns away from the tag.
    private static final double KP_TURN = 0.02;            // Turn power per degree of yaw.
    private static final double MIN_TURN_POWER = 0.08;     // Overcomes carpet friction near center.
    private static final double MAX_TURN_POWER = 0.6;      // Cap on turn speed.
    private static final double ALIGN_DEADBAND_DEG = 1.0;  // Within this many degrees = aligned.
    private static final double TRIGGER_THRESHOLD = 0.5;   // Trigger past this = aligning.

    private final FearOfBot robot;
    private final Limelight3A limelight;

    private boolean lastX;

    // Runs once when INIT is pressed.
    public AlignTestTeleOp(FearOfBot robot) {
        super(robot);
        this.robot = robot;

        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.a().onTrue(robot.intake.toggleWith(robot.transfer));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    // Runs once when PLAY is pressed.
    @Override
    public void start() {
        robot.drivetrain.init(hardwareMap);
        robot.outtake.begin();
        robot.intake.turnOn().schedule();
        robot.transfer.setOn(true);
    }

    // Runs every loop while PLAY is active.
    @Override
    public void periodic() {
        boolean xPressed = gamepad1.x && !lastX;
        lastX = gamepad1.x;
        if (xPressed) robot.outtake.shoot().schedule();

        // ---- Yaw of the tag (NaN if no tag is seen) ----
        LLResult result = limelight.getLatestResult();
        double tagYaw = Double.NaN;
        if (result != null && result.isValid()) {
            tagYaw = firstTagYawDeg(result);
        }

        // ---- Drive, with align on the right trigger ----
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        String mode = "DRIVING";
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            if (Double.isNaN(tagYaw)) {
                mode = "ALIGNING - NO TAG";
                turn = 0.0;
            } else if (Math.abs(tagYaw) <= ALIGN_DEADBAND_DEG) {
                mode = "ALIGNED";
                turn = 0.0;
            } else {
                mode = "ALIGNING";
                turn = computeTurnPower(TURN_SIGN * tagYaw);
            }
        }

        robot.drivetrain.drive(forward, strafe, turn);

        // ---- Telemetry ----
        telemetry.addData("Mode", mode);
        telemetry.addData("Tag yaw (deg, 0 = square)", Double.isNaN(tagYaw) ? "NONE" : String.format("%.2f", tagYaw));
        telemetry.addData("Turn power", "%.2f", turn);
        telemetry.addData("Limelight valid", result != null && result.isValid());
        telemetry.addData("Pipeline index", result == null ? "--" : result.getPipelineIndex());
    }

    // Runs once when STOP is pressed.
    @Override
    public void end() {
        robot.drivetrain.stopNow();
        robot.outtake.stopNow();
        robot.intake.stopNow();
        robot.transfer.stopNow();
        limelight.stop();
    }

    /** Turn power toward the error, with a small push so the robot doesn't stall short of center. */
    private static double computeTurnPower(double errorDeg) {
        double power = KP_TURN * errorDeg;
        power += Math.signum(power) * MIN_TURN_POWER;
        return Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    /** Yaw (degrees) of the first AprilTag relative to the camera, or NaN if there is no tag pose. */
    private static double firstTagYawDeg(LLResult result) {
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return Double.NaN;
        Pose3D pose = result.getFiducialResults().get(0).getTargetPoseCameraSpace();
        if (pose == null || pose.getOrientation() == null) return Double.NaN;
        return pose.getOrientation().getYaw(AngleUnit.DEGREES);
    }
}
