package org.firstinspires.ftc.teamcode.teleopVariants;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.teleOp.FearOfBot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import dev.nextftc.robot.opmode.NextOpMode;   // Base class for NextFTC OpModes.
import dev.nextftc.robot.opmode.NextTeleop;   // Puts this OpMode in the Driver Station's TeleOp list.
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.units.Units;               // Converts RPM to the units the shooter motor uses.

/*
 * MECHANISMS SHOT LOG TELEOP (NextFTC)
 *
 * Same as MechanismsTeleOp, plus one change: every shot is logged to a CSV file.
 *
 * CONTROLS (gamepad 1):
 *   Left stick   = drive / strafe            Right stick = turn (only when NOT aligning)
 *   X            = SHOOT: auto-align to the tag, then open the gate
 *   A            = intake + transfer on / off together
 *   Left bumper  = Limelight pipeline RED <-> BLUE
 *
 * WHAT A SHOT DOES:
 *   1. X pressed -> ALIGNING. The robot turns itself (tx from the Limelight) until the tag
 *      is within ALIGN_DEADBAND_DEG, or ALIGN_TIMEOUT_MS passes with no alignment.
 *   2. At that moment the shot is captured: yaw, odometry X/Y, linked tag, alignment time,
 *      projected RPM, and the actual shooter RPM. The gate opens (same as Outtake.shoot()).
 *   3. When the gate closes (Outtake.GATE_OPEN_MS later) the actual RPM is read again and
 *      one CSV row is written.
 *
 * LOG FILE:
 *   /sdcard/FIRST/ShotLogs/shots_<date_time>.csv (one file per PLAY). Each row is also sent to
 *   the RobotLog, so it shows up in logcat if the file is not reachable.
 *
 * CSV COLUMNS:
 *   shot, timestamp_ms, linked_tag_id, yaw_deg, odo_x_in, odo_y_in, align_time_ms, aligned,
 *   projected_rpm, rpm_gate_open, rpm_gate_close
 *
 *   projected_rpm = Outtake.TARGET_RPM (the speed the shooter is commanded to hold).
 *   aligned       = true if the tag was centered before the shot, false if the timeout fired.
 *   Missing values are written as NaN (no tag, or no Pinpoint in the config).
 */
@NextTeleop(name = "Mechanisms Shot Log TeleOp")
public class MechanismsShotLogTeleOp extends NextOpMode {

    // ---- Auto-align (same tuning as LimelightAlignTeleOp) ----
    private static final double KP_TURN = 0.02;            // Power per degree of tx.
    private static final double MIN_TURN_POWER = 0.08;     // Overcomes carpet friction near center.
    private static final double MAX_TURN_POWER = 0.6;      // Cap so the robot doesn't whip around.
    private static final double ALIGN_DEADBAND_DEG = 1.0;  // Within this many degrees = aligned.
    private static final long ALIGN_TIMEOUT_MS = 2000;     // Give up aligning and shoot after this long.

    // ---- Limelight pipelines ----
    private static final int PIPELINE_RED = 1;
    private static final int PIPELINE_BLUE = 2;

    private static final String LOG_DIR = "/sdcard/FIRST/ShotLogs";
    private static final String LOG_HEADER =
            "shot,timestamp_ms,linked_tag_id,yaw_deg,odo_x_in,odo_y_in,align_time_ms,aligned,"
                    + "projected_rpm,rpm_gate_open,rpm_gate_close";

    private enum ShotState { IDLE, ALIGNING, GATE_OPEN }

    private final FearOfBot robot;
    private final Limelight3A limelight;
    private final GoBildaPinpointDriver pinpoint; // null if the Pinpoint is missing from the config.

    private int currentPipeline = PIPELINE_RED;
    private boolean lastX, lastLeftBumper;

    private ShotState state = ShotState.IDLE;
    private long shotPressMs;   // When X was pressed (alignment starts).
    private long gateOpenMs;    // When the gate opened.

    // Values captured for the shot in progress, written to the log when the gate closes.
    private int shotNumber = 0;
    private int linkedTagId = -1;
    private double yawDeg = Double.NaN;
    private double odoX = Double.NaN;
    private double odoY = Double.NaN;
    private double alignTimeMs = Double.NaN;
    private boolean aligned = false;
    private double rpmGateOpen = Double.NaN;

    private PrintWriter logWriter;
    private String logPath = "not opened";
    private String lastShot = "none yet";

    // Runs once when INIT is pressed. Same hardware setup as MechanismsTeleOp, plus the Limelight and Pinpoint.
    public MechanismsShotLogTeleOp(FearOfBot robot) {
        super(robot);
        this.robot = robot;

        // Keep the intake toggle from MechanismsTeleOp.
        CommandGamepad driver = new CommandGamepad(gamepad1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(currentPipeline);

        GoBildaPinpointDriver foundPinpoint = null;
        try {
            foundPinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (RuntimeException notInConfig) {
            // Leave it null. Yaw and position are logged as NaN.
        }
        pinpoint = foundPinpoint;
    }

    // Runs once when PLAY is pressed.
    @Override
    public void start() {
        robot.drivetrain.init(hardwareMap);
        robot.outtake.begin();             // Gate to rest, shooter to Outtake.TARGET_RPM.
        robot.intake.turnOn().schedule();
        robot.transfer.setOn(true);
        openLogFile();
    }

    // Runs every loop while PLAY is active.
    @Override
    public void periodic() {
        if (pinpoint != null) pinpoint.update();

        // ---- Button presses (each one counts once) ----
        boolean xPressed = gamepad1.x && !lastX;
        boolean leftBumperPressed = gamepad1.left_bumper && !lastLeftBumper;
        lastX = gamepad1.x;
        lastLeftBumper = gamepad1.left_bumper;

        if (leftBumperPressed) switchPipeline();

        // ---- Vision ----
        LLResult result = limelight.getLatestResult();
        boolean tagVisible = result != null
                && result.isValid()
                && result.getPipelineIndex() == currentPipeline;
        double tx = tagVisible ? result.getTx() : 0.0;
        int visibleTagId = tagVisible ? firstTagId(result) : -1;

        // ---- Driving: sticks for forward/strafe; turn is set by the aligner while shooting ----
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        long now = System.currentTimeMillis();

        if (state == ShotState.IDLE && xPressed) {
            state = ShotState.ALIGNING;
            shotPressMs = now;
        }

        if (state == ShotState.ALIGNING) {
            if (tagVisible) linkedTagId = visibleTagId;
            boolean centered = tagVisible && Math.abs(tx) <= ALIGN_DEADBAND_DEG;
            boolean timedOut = now - shotPressMs >= ALIGN_TIMEOUT_MS;

            if (centered || timedOut) {
                turn = 0.0;
                fireShot(now, centered);
            } else {
                turn = tagVisible ? computeTurnPower(tx) : 0.0; // Don't spin blind if the tag is gone.
            }
        }

        robot.drivetrain.drive(forward, strafe, turn);

        // ---- Gate closes: finish the shot and log it ----
        if (state == ShotState.GATE_OPEN && now - gateOpenMs >= Outtake.GATE_OPEN_MS) {
            double rpmGateClose = actualRpm();
            writeShotRow(rpmGateClose);
            state = ShotState.IDLE;
        }

        // ---- Telemetry ----
        double actualRpm = actualRpm();
        telemetry.addData("State", state.name());
        telemetry.addData("Shooter target RPM", "%.0f", Outtake.TARGET_RPM);
        telemetry.addData("Shooter actual RPM", "%.0f", actualRpm);
        telemetry.addData("Pipeline", pipelineName(currentPipeline));
        telemetry.addData("Tag", tagVisible ? "FOUND (id " + visibleTagId + ")" : "NONE");
        telemetry.addData("tx (deg)", tagVisible ? String.format(Locale.US, "%.2f", tx) : "--");
        telemetry.addData("Yaw (deg)", Double.isNaN(readHeadingDeg()) ? "NO PINPOINT"
                : String.format(Locale.US, "%.2f", readHeadingDeg()));
        telemetry.addData("Last shot", lastShot);
        telemetry.addData("Log file", logPath);
        telemetry.addData("Controls", "X shoot | A intake | LB pipeline");
    }

    // Runs once when STOP is pressed. Turn everything off and close the log.
    @Override
    public void end() {
        robot.drivetrain.stopNow();
        robot.outtake.stopNow();
        robot.intake.stopNow();
        robot.transfer.stopNow();
        limelight.pipelineSwitch(PIPELINE_RED);
        limelight.stop();
        if (logWriter != null) logWriter.close();
    }

    // ---- Shot helpers ----

    /** Captures the shot's pose, tag, and alignment time, then opens the gate. */
    private void fireShot(long now, boolean centered) {
        shotNumber++;
        aligned = centered;
        alignTimeMs = now - shotPressMs;
        yawDeg = readHeadingDeg();
        odoX = pinpoint != null ? pinpoint.getPosX(DistanceUnit.INCH) : Double.NaN;
        odoY = pinpoint != null ? pinpoint.getPosY(DistanceUnit.INCH) : Double.NaN;
        rpmGateOpen = actualRpm();

        robot.outtake.shoot().schedule(); // Gate opens now, closes after Outtake.GATE_OPEN_MS.
        gateOpenMs = now;
        state = ShotState.GATE_OPEN;
    }

    /** Writes one CSV row with the values captured in fireShot(). */
    private void writeShotRow(double rpmGateClose) {
        String row = String.format(Locale.US,
                "%d,%d,%d,%.3f,%.3f,%.3f,%.0f,%b,%.0f,%.0f,%.0f",
                shotNumber, gateOpenMs, linkedTagId, yawDeg, odoX, odoY, alignTimeMs, aligned,
                Outtake.TARGET_RPM, rpmGateOpen, rpmGateClose);

        if (logWriter != null) {
            logWriter.println(row);
            logWriter.flush();
        }
        RobotLog.ii("ShotLog", row);
        lastShot = String.format(Locale.US, "#%d tag=%d align=%.0fms rpm %.0f->%.0f",
                shotNumber, linkedTagId, alignTimeMs, rpmGateOpen, rpmGateClose);
    }

    /** Opens a new CSV file for this PLAY session. If that fails, shots still go to the RobotLog. */
    private void openLogFile() {
        try {
            File dir = new File(LOG_DIR);
            dir.mkdirs();
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File file = new File(dir, "shots_" + stamp + ".csv");
            logWriter = new PrintWriter(new FileWriter(file, true));
            logWriter.println(LOG_HEADER);
            logWriter.flush();
            logPath = file.getAbsolutePath();
        } catch (IOException e) {
            logWriter = null;
            logPath = "FAILED - see RobotLog";
        }
    }

    // ---- Helpers ----

    private double actualRpm() {
        return robot.outtake.shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    /** Robot heading (yaw) in degrees from the Pinpoint, or NaN if it is not there. */
    private double readHeadingDeg() {
        if (pinpoint == null) return Double.NaN;
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    private void switchPipeline() {
        currentPipeline = (currentPipeline == PIPELINE_RED) ? PIPELINE_BLUE : PIPELINE_RED;
        limelight.pipelineSwitch(currentPipeline);
    }

    /** ID of the first AprilTag in the result, or -1 if none. */
    private static int firstTagId(LLResult result) {
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return -1;
        return result.getFiducialResults().get(0).getFiducialId();
    }

    /** Proportional turn toward the tag, with a small push so the robot doesn't stall short of center. */
    private static double computeTurnPower(double tx) {
        double power = KP_TURN * tx;
        power += Math.signum(power) * MIN_TURN_POWER;
        return Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    private static String pipelineName(int pipeline) {
        return pipeline == PIPELINE_RED ? "1 - RED" : "2 - BLUE";
    }
}
