package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.telemetry.TelemetryManager;      // The Panels object the robot hands us to fill.
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import dev.nextftc.robot.Mechanism;

/*
 * LIMELIGHT MECHANISM (AprilTag pipeline 1)
 *
 * HARDWARE (name must match the robot configuration on the Driver Station):
 *   limelight = Limelight 3A
 *
 * HOW IT WORKS:
 *   - init(hardwareMap) starts the Limelight and switches it to pipeline 1 (AprilTag).
 *   - periodic() reads the latest result every loop: tag yaw, hive center bearing, and hive distance.
 *   - turnPower(aligning, driverTurn) returns the turn to use: the align turn while aligning,
 *     otherwise the driver's turn stick.
 *   - Distance is the straight-line distance from the camera to the hive center.
 *
 * If it turns the wrong way, set TURN_SIGN to -1.0.
 */
public class Limelight implements Mechanism {

    // ---- Settings ----
    public static double TURN_SIGN = 1.0;              // 1.0 if the robot turns away from the tag.
    public static double TARGET_YAW_DEG = 0.0;         // The tag yaw the robot should end up at (0 = square).
    // true = turn until the hive CENTER is straight ahead. false = turn to TARGET_YAW_DEG.
    public static boolean ALIGN_TO_HIVE_CENTER = true;
    // -1.0 if moving along the hive face goes the wrong way for the offsets below.
    public static double FACE_SIGN = 1.0;
    public static double KP_TURN = 0.02;              // Turn power per degree of error.
    public static double MIN_TURN_POWER = 0.08;       // Overcomes carpet friction near center.
    public static double MAX_TURN_POWER = 0.6;        // Cap on turn speed.
    public static double ALIGN_DEADBAND_DEG = 1.0;    // Within this many degrees = aligned.

    // ---- Hardware ----
    private Limelight3A limelight;

    // ---- Latest readings (updated in periodic) ----
    private LLResult result;
    private double tagYawDeg = Double.NaN;
    private double centerBearingDeg = Double.NaN;   // + = hive center is to the right.
    private double hiveDistanceIn = Double.NaN;     // Camera to hive center, inches.
    private String mode = "DRIVING";

    /** Starts the Limelight and switches it to pipeline 1 (AprilTag). Call once from the OpMode constructor. */
    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1); // Pipeline 1 = AprilTag.
    }

    /** Stops the Limelight right now. Use this in an OpMode's end(). */
    public void stopNow() {
        if (limelight != null) limelight.stop();
    }

    /**
     * The turn power to use this loop.
     * While aligning, the robot turns toward the hive (ignores driverTurn).
     * Otherwise it returns driverTurn unchanged.
     */
    public double turnPower(boolean aligning, double driverTurn) {
        if (!aligning) {
            mode = "DRIVING";
            return driverTurn;
        }
        boolean useCenter = ALIGN_TO_HIVE_CENTER && !Double.isNaN(centerBearingDeg);
        double error = useCenter ? centerBearingDeg : (tagYawDeg - TARGET_YAW_DEG);
        if (Double.isNaN(error)) {
            mode = "ALIGNING - NO TAG";
            return 0.0;
        }
        if (Math.abs(error) <= ALIGN_DEADBAND_DEG) {
            mode = "ALIGNED";
            return 0.0;
        }
        mode = "ALIGNING";
        return computeTurnPower(TURN_SIGN * error);
    }

    public String mode() { return mode; }
    public double tagYawDeg() { return tagYawDeg; }
    public double centerBearingDeg() { return centerBearingDeg; }
    public double hiveDistanceCm() { return hiveDistanceIn * 2.54; }   // NaN if no hive tag.
    public boolean resultValid() { return result != null && result.isValid(); }
    public int pipelineIndex() { return result == null ? -1 : (int) result.getPipelineIndex(); }

    /** Limelight readouts (align mode, tag status, hive distance) for Panels and the Driver Station. Prints nothing until init(). */
    public void addTelemetry(TelemetryManager panels) {
        if (limelight == null) return; // init() has not run in this OpMode.
        panels.addData("Align mode", mode);
        panels.addData("Limelight valid", resultValid());
        panels.addData("Pipeline index", pipelineIndex());
        double distanceCm = hiveDistanceCm();
        panels.addData("Distance (cm)", Double.isNaN(distanceCm) ? "--" : String.valueOf(Math.round(distanceCm)));
    }

    // Runs every loop automatically (FearOfBot lists this mechanism).
    @Override
    public void periodic() {
        if (limelight == null) return; // init() has not been called in this OpMode.

        result = limelight.getLatestResult();
        tagYawDeg = Double.NaN;
        centerBearingDeg = Double.NaN;
        hiveDistanceIn = Double.NaN;

        if (result != null && result.isValid()) {
            tagYawDeg = averageTagYawDeg(result);
            double[] hiveCenter = hiveCenterCameraIn(result);
            if (hiveCenter != null) {
                centerBearingDeg = Math.toDegrees(Math.atan2(hiveCenter[0], hiveCenter[2]));
                hiveDistanceIn = Math.sqrt(
                        hiveCenter[0] * hiveCenter[0]
                                + hiveCenter[1] * hiveCenter[1]
                                + hiveCenter[2] * hiveCenter[2]);
            }
        }
    }

    /** Turn power toward the error, with a small push so the robot doesn't stall short of center. */
    private static double computeTurnPower(double errorDeg) {
        double power = KP_TURN * errorDeg;
        power += Math.signum(power) * MIN_TURN_POWER;
        return Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER);
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
     * Returns null if no hive tag has a pose.
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
}
