package org.firstinspires.ftc.teamcode.teleopVariants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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
 *   X             = shoot (gate opens, same as MechanismsTeleOp)
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
@NextTeleop(name = "Align Test TeleOp")
public class AlignTestTeleOp extends NextOpMode {

    private static final double TURN_SIGN = 1.0;          // 1.0 if the robot turns away from the tag.
    private static final double TARGET_YAW_DEG = 0.0;      // The tag yaw the robot should end up at (0 = square).
    // true = turn until the hive CENTER is straight ahead. false = turn to TARGET_YAW_DEG (the old behavior).
    private static final boolean ALIGN_TO_HIVE_CENTER = true;
    // -1.0 if moving along the hive face goes the wrong way for the offsets below.
    private static final double FACE_SIGN = 1.0;
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

        // Intake toggle, same as MechanismsTeleOp.
        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.a().onTrue(robot.intake.toggleWith(robot.transfer));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1); // Always use pipeline 1 (AprilTag) when the OpMode starts.
    }

    // Runs once when PLAY is pressed.
    @Override
    public void start() {
        robot.drivetrain.init(hardwareMap);
        robot.outtake.begin();              // Gate to rest, shooter to Outtake.TARGET_RPM.
        robot.intake.turnOn().schedule();   // Intake runs from the start, same as MechanismsTeleOp.
        robot.transfer.setOn(true);
    }

    // Runs every loop while PLAY is active.
    @Override
    public void periodic() {
        // ---- Shoot (each press counts once) ----
        boolean xPressed = gamepad1.x && !lastX;
        lastX = gamepad1.x;
        if (xPressed) robot.outtake.shoot().schedule();

        // ---- Yaw of the tags, and the hive center (NaN if no hive tag is seen) ----
        LLResult result = limelight.getLatestResult();
        double tagYaw = Double.NaN;
        double[] hiveCenter = null; // camera-space x, y, z in inches
        if (result != null && result.isValid()) {
            tagYaw = averageTagYawDeg(result);
            hiveCenter = hiveCenterCameraIn(result);
        }
        double centerBearingDeg = hiveCenter == null
                ? Double.NaN
                : Math.toDegrees(Math.atan2(hiveCenter[0], hiveCenter[2])); // + = center is to the right

        // ---- Drive, with align on the right trigger ----
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        String mode = "DRIVING";
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            boolean useCenter = ALIGN_TO_HIVE_CENTER && !Double.isNaN(centerBearingDeg);
            double error = useCenter ? centerBearingDeg : (tagYaw - TARGET_YAW_DEG);
            if (Double.isNaN(error)) {
                mode = "ALIGNING - NO TAG";
                turn = 0.0;
            } else if (Math.abs(error) <= ALIGN_DEADBAND_DEG) {
                mode = "ALIGNED";
                turn = 0.0;
            } else {
                mode = "ALIGNING";
                turn = computeTurnPower(TURN_SIGN * error);
            }
        }

        robot.drivetrain.drive(forward, strafe, turn);

        // ---- Telemetry ----
        telemetry.addData("Mode", mode);
        telemetry.addData("Tag yaw (deg, 0 = square)", Double.isNaN(tagYaw) ? "NONE" : String.format("%.2f", tagYaw));
        telemetry.addData("Turn power", "%.2f", turn);
        telemetry.addData("Limelight valid", result != null && result.isValid());
        telemetry.addData("Tags seen", result == null || result.getFiducialResults() == null
                ? 0 : result.getFiducialResults().size());
        telemetry.addData("Hive center tx (deg, 0 = centered)", result == null || !result.isValid()
                ? "NONE" : String.format("%.2f", averageTagTxDeg(result)));
        telemetry.addData("Hive center bearing (deg, 0 = centered)", Double.isNaN(centerBearingDeg)
                ? "NONE" : String.format("%.2f", centerBearingDeg));
        if (hiveCenter != null) {
            telemetry.addData("Hive center x, z (in)", String.format("%.1f, %.1f", hiveCenter[0], hiveCenter[2]));
        }
        telemetry.addData("Pipeline index", result == null ? "--" : result.getPipelineIndex());
        telemetry.addData("Tag pose (x,y,z in | yaw,pitch,roll deg)", poseText(result));
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

    /** Full pose of the first AprilTag in camera space, as text for telemetry. */
    private static String poseText(LLResult result) {
        if (result == null || result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return "NO TAG";
        }
        Pose3D pose = result.getFiducialResults().get(0).getTargetPoseCameraSpace();
        if (pose == null || pose.getPosition() == null || pose.getOrientation() == null) {
            return "NO POSE";
        }
        org.firstinspires.ftc.robotcore.external.navigation.Position p =
                pose.getPosition().toUnit(DistanceUnit.INCH);
        return String.format("%.2f, %.2f, %.2f | %.2f, %.2f, %.2f",
                p.x, p.y, p.z,
                pose.getOrientation().getYaw(AngleUnit.DEGREES),
                pose.getOrientation().getPitch(AngleUnit.DEGREES),
                pose.getOrientation().getRoll(AngleUnit.DEGREES));
    }

    /**
     * Offset (inches) of a hive tag from the hive cluster center, measured along the face.
     * Positive = right of center when looking at the hive. NaN if the tag is not a hive tag.
     * Values are from the 2026 field figure (tags 20-23, 3.75 in apart, cluster center between 21 and 22).
     */
    private static double hiveTagOffsetIn(int id) {
        switch (id) {
            case 34: return -6.5;
            case 35: return -2.75;
            case 36: return 2.75;
            case 37: return 6.5;
            default: return Double.NaN;
        }
    }

    /**
     * Estimated hive cluster center in camera space (x, y, z, inches), averaged over every visible hive tag.
     * Each tag gives its own estimate by moving along the face by its known offset. Returns null if no hive tag has a pose.
     */
    private static double[] hiveCenterCameraIn(LLResult result) {
        if (result.getFiducialResults() == null) return null;
        double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            double offset = hiveTagOffsetIn(tag.getFiducialId());
            if (Double.isNaN(offset)) continue;
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null || pose.getPosition() == null || pose.getOrientation() == null) continue;

            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            double yawRad = Math.toRadians(pose.getOrientation().getYaw(AngleUnit.DEGREES));
            // Unit vector along the hive face (the tag's horizontal axis), in camera space.
            double dirX = Math.cos(yawRad);
            double dirZ = -Math.sin(yawRad);

            // Center = tag position - offset * face direction.
            sumX += p.x - FACE_SIGN * offset * dirX;
            sumY += p.y;
            sumZ += p.z - FACE_SIGN * offset * dirZ;
            count++;
        }
        if (count == 0) return null;
        return new double[]{sumX / count, sumY / count, sumZ / count};
    }

    /** Average yaw (degrees) of all visible AprilTags relative to the camera, or NaN if none has a pose. */
    private static double averageTagYawDeg(LLResult result) {
        if (result.getFiducialResults() == null) return Double.NaN;
        double sum = 0.0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null || pose.getOrientation() == null) continue;
            sum += pose.getOrientation().getYaw(AngleUnit.DEGREES);
            count++;
        }
        return count == 0 ? Double.NaN : sum / count;
    }

    /** Average horizontal angle (tx, degrees) to all visible AprilTags. This is where the hive center is. */
    private static double averageTagTxDeg(LLResult result) {
        if (result.getFiducialResults() == null) return Double.NaN;
        double sum = 0.0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            sum += tag.getTargetXDegrees();
            count++;
        }
        return count == 0 ? Double.NaN : sum / count;
    }
}
