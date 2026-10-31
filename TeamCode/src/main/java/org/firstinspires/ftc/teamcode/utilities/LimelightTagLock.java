package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.limelightvision.LLResult;          // One frame of Limelight data.
import com.qualcomm.hardware.limelightvision.LLResultTypes;     // AprilTag (fiducial) results.
import com.qualcomm.hardware.limelightvision.Limelight3A;       // The Limelight camera itself.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;          // Basic FTC OpMode (init/loop/stop).
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;          // Puts this OpMode in the TeleOp list.
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;                 // Timer for "how long has the tag been lost".

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/*
 * LIMELIGHT APRILTAG LOCK (Limelight named "limelight")
 *
 * This is a TeleOp OpMode. On the Driver Station it is found in the TeleOp list.
 *
 * WHAT IT DOES:
 *   - Looks for AprilTags 38, 39, 40, 41, 42, 43, 44, 45.
 *   - Locks on to the first one it finds and STAYS on it, even if other tags are in view.
 *   - If the locked tag is gone for longer than LOST_TIMEOUT_MS, it locks on to the first
 *     tag from the list that it can see.
 *   - Shows the distance to the locked tag, updating as fast as the Limelight sends frames.
 *
 * HOW TO USE:
 *   1. Make sure your robot configuration has a Limelight named "limelight".
 *   2. PIPELINE must be an AprilTag pipeline with "Full 3D" turned on in the Limelight web
 *      page (otherwise the distance is 0).
 *   3. Select "Limelight Tag Lock" and press START.
 *   4. Gamepad 1 A = drop the current lock and grab the first tag it sees.
 */
@Utility(name = "Limelight Tag Lock")
public class LimelightTagLock extends OpMode {

    // ---- Settings ----
    public static String LIMELIGHT_NAME = "limelight";  // Limelight's name in the robot configuration.
    public static int PIPELINE = 0;                     // AprilTag pipeline number on the Limelight.
    public static int POLL_RATE_HZ = 100;               // How often we ask the Limelight for data (max 100).
    public static double LOST_TIMEOUT_MS = 150;         // Tag must be gone this long before we switch.
    // Stops one missed frame from changing the lock.
    public static final int[] TAG_IDS = {38, 39, 40, 41, 42, 43, 44, 45};

    private static final double METERS_TO_INCHES = 39.3701;
    private static final int NO_TAG = -1;

    // ---- State ----
    private Limelight3A limelight;
    private int lockedId = NO_TAG;                      // Tag we are locked on to, or NO_TAG.
    private final ElapsedTime lostTimer = new ElapsedTime(); // Time since the locked tag was last seen.
    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean lastA = false;

    // Last good reading of the locked tag (kept while the tag is briefly lost).
    private double distanceIn = 0;         // Straight-line distance, camera to tag.
    private double groundDistanceIn = 0;   // Distance along the floor (ignores height difference).
    private double xIn = 0, yIn = 0, zIn = 0;
    private double txDeg = 0, tyDeg = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start(); // Without start(), getLatestResult() returns nothing.

        // Send telemetry as fast as possible so the numbers update almost instantly.
        telemetry.setMsTransmissionInterval(11);

        telemetry.addData(">", "Limelight ready. Press START.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // A = drop the lock and take the first tag in view.
        boolean a = gamepad1.a;
        if (a && !lastA) {
            lockedId = NO_TAG;
        }
        lastA = a;

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> tags = null;
        if (result != null && result.isValid()) {
            tags = result.getFiducialResults();
        }

        // 1. Is the tag we are locked on to in this frame?
        LLResultTypes.FiducialResult target = findTag(tags, lockedId);

        if (target != null) {
            lostTimer.reset();
        } else if (lockedId == NO_TAG || lostTimer.milliseconds() > LOST_TIMEOUT_MS) {
            // 2. No lock, or the locked tag has been gone too long: lock the first tag we see.
            target = firstAllowedTag(tags);
            if (target != null) {
                lockedId = target.getFiducialId();
                lostTimer.reset();
            } else {
                lockedId = NO_TAG;
            }
        }
        // 3. Otherwise the tag is only briefly lost: keep the lock and the last reading.

        if (target != null) {
            updateDistance(target);
        }

        boolean seenNow = target != null;
        telemetry.addData("Locked Tag", lockedId == NO_TAG ? "NONE (searching)" : lockedId);
        telemetry.addData("Status", seenNow ? "TRACKING"
                : lockedId != NO_TAG ? String.format("LOST %.0f ms (holding)", lostTimer.milliseconds())
                : "No tag from list in view");
        if (lockedId != NO_TAG) {
            telemetry.addData("Distance", "%.1f in (%.2f m)", distanceIn, distanceIn / METERS_TO_INCHES);
            telemetry.addData("Floor Distance", "%.1f in", groundDistanceIn);
            telemetry.addData("Tag X/Y/Z (cam)", "%.1f / %.1f / %.1f in", xIn, yIn, zIn);
            telemetry.addData("tx / ty", "%.2f° / %.2f°", txDeg, tyDeg);
        }
        telemetry.addData("Tags in view", tagList(tags));
        if (result != null) {
            telemetry.addData("Data Age", "%d ms", result.getStaleness());
            telemetry.addData("LL Latency", "%.0f ms", result.getCaptureLatency() + result.getTargetingLatency());
        }
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.addLine("A = re-pick tag");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    // Returns the tag with this id from the frame, or null.
    private LLResultTypes.FiducialResult findTag(List<LLResultTypes.FiducialResult> tags, int id) {
        if (tags == null || id == NO_TAG) return null;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == id) return tag;
        }
        return null;
    }

    // Returns the first tag in the frame whose id is in TAG_IDS, or null.
    private LLResultTypes.FiducialResult firstAllowedTag(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null) return null;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (isAllowed(tag.getFiducialId())) return tag;
        }
        return null;
    }

    private boolean isAllowed(int id) {
        for (int allowed : TAG_IDS) {
            if (allowed == id) return true;
        }
        return false;
    }

    // Camera space: x = right, y = down, z = forward (meters).
    private void updateDistance(LLResultTypes.FiducialResult tag) {
        Position p = tag.getTargetPoseCameraSpace().getPosition();
        double x = p.x, y = p.y, z = p.z;
        xIn = x * METERS_TO_INCHES;
        yIn = y * METERS_TO_INCHES;
        zIn = z * METERS_TO_INCHES;
        distanceIn = Math.sqrt(x * x + y * y + z * z) * METERS_TO_INCHES;
        groundDistanceIn = Math.hypot(x, z) * METERS_TO_INCHES;
        txDeg = tag.getTargetXDegrees();
        tyDeg = tag.getTargetYDegrees();
    }

    private String tagList(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null || tags.isEmpty()) return "none";
        StringBuilder sb = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : tags) {
            if (sb.length() > 0) sb.append(", ");
            sb.append(tag.getFiducialId());
        }
        return sb.toString();
    }
}
