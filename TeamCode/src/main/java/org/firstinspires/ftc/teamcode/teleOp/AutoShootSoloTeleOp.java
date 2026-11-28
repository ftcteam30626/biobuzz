package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mechanisms.Limelight;

/*
 * AUTO SHOOT SOLO TELEOP
 *
 * A standalone OpMode. It does NOT use FearOfBot or NextFTC: it has its own motors and gate,
 * and only reuses the Limelight class for the hive alignment and distance.
 *
 * CONTROLS (gamepad 1):
 *   Left stick  = drive / strafe,  Right stick = turn (ignored while aligning)
 *   LT (hold)   = align to the hive with the Limelight
 *   X           = auto shot: set RPM from the hive distance, spin up, fire
 *   A           = intake + transfer on / off
 *
 * SHOT SEQUENCE (X):
 *   1. Distance = hive distance now, or the last one seen if no tag is visible.
 *   2. RPM = RPM_PER_CM * distance + RPM_AT_ZERO, kept between MIN_RPM and MAX_RPM.
 *   3. Shooter spins to that RPM. When it is within RPM_TOLERANCE (or SPIN_UP_TIMEOUT_MS passes),
 *      the gate opens for GATE_OPEN_MS and the ball fires.
 *
 * HARDWARE (names must match the robot configuration):
 *   leftFront, leftBack, rightFront, rightBack = drive motors
 *   IntakeMotor, TransferMotor                 = intake and transfer
 *   ShooterMotor                               = shooter (velocity control, 28 ticks per rev)
 *   GateServo                                  = gate
 *   limelight                                  = Limelight 3A
 */
@TeleOp(name = "Auto Shoot Solo", group = "Shooter")
public class AutoShootSoloTeleOp extends OpMode {

    // ---- Shooter and gate ----
    private static final double TICKS_PER_REV = 28.0;   // goBILDA bare motor. Change if the motor differs.
    private static final double GATE_START = 0.75;      // Gate closed (rest).
    private static final double GATE_SHOOT = 0.5;       // Gate open (shooting).
    private static final long GATE_OPEN_MS = 3000;      // How long the gate stays open per shot.

    // ---- RPM formula: RPM = RPM_PER_CM * distance(cm) + RPM_AT_ZERO (fit from test data) ----
    private static final double RPM_PER_CM = 3.866;
    private static final double RPM_AT_ZERO = 1387.47;
    private static final double MIN_RPM = 1900;         // Lowest tested RPM.
    private static final double MAX_RPM = 2200;         // Highest tested RPM. Safety cap beyond that.
    private static final double START_RPM = 2000;      // Shooter speed as soon as PLAY starts, before any shot.
    private static final double RPM_TOLERANCE = 50;     // Fire once the shooter is this close to the target.
    private static final long SPIN_UP_TIMEOUT_MS = 3000; // Fire anyway after this long.

    // ---- Intake, transfer, align ----
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 0.75;
    private static final double TRIGGER_THRESHOLD = 0.5;

    private enum ShotState { IDLE, SPINNING_UP, FIRING }

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private DcMotor intakeMotor, transferMotor;
    private DcMotorEx shooterMotor;
    private Servo gateServo;
    private final Limelight limelight = new Limelight();

    private ShotState state = ShotState.IDLE;
    private long stateStartMs;
    private double targetRpm = 0;
    private double lastDistanceCm = Double.NaN;
    private String lastShot = "none yet";
    private boolean lastX, lastA, intakeOn;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor m : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Intake runs reversed, same as Intake.java.
        transferMotor = hardwareMap.get(DcMotor.class, "TransferMotor");
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gateServo = hardwareMap.get(Servo.class, "GateServo");
        gateServo.setPosition(GATE_START);

        limelight.init(hardwareMap); // Pipeline 1 (AprilTag).
    }

    @Override
    public void start() {
        targetRpm = START_RPM;
        shooterMotor.setVelocity(rpmToTicksPerSec(START_RPM)); // Shooter spins up to 2000 RPM when PLAY starts.
        intakeOn = true;
        intakeMotor.setPower(INTAKE_POWER);
        transferMotor.setPower(TRANSFER_POWER);
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        limelight.periodic(); // Reads the latest Limelight result.

        // ---- Drive, with LT to align ----
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        boolean aligning = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        double turn = limelight.turnPower(aligning, gamepad1.right_stick_x);
        drive(forward, strafe, turn);

        // ---- A toggles intake + transfer ----
        if (gamepad1.a && !lastA) toggleIntake();
        lastA = gamepad1.a;

        // ---- Remember the last hive distance ----
        double visibleDistanceCm = limelight.hiveDistanceCm();
        if (!Double.isNaN(visibleDistanceCm)) lastDistanceCm = visibleDistanceCm;

        // ---- X starts one shot ----
        if (gamepad1.x && !lastX && state == ShotState.IDLE) startShot(visibleDistanceCm, now);
        lastX = gamepad1.x;

        updateShot(now);

        telemetry.addData("Shot state", state.name());
        telemetry.addData("Last shot", lastShot);
        telemetry.addData("Distance (cm)", Double.isNaN(lastDistanceCm) ? "NO TAG YET" : String.format("%.0f", lastDistanceCm));
        telemetry.addData("Target RPM", String.format("%.0f", targetRpm));
        telemetry.addData("Shooter RPM", String.format("%.0f", actualRpm()));
        telemetry.addData("Align mode", limelight.mode());
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.update();
    }

    @Override
    public void stop() {
        drive(0, 0, 0);
        shooterMotor.setVelocity(0);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        limelight.stopNow();
    }

    // ---- Shot sequence ----

    /** Sets the shooter RPM for the distance and starts spinning up. Uses the last distance if no tag is visible now. */
    private void startShot(double visibleDistanceCm, long now) {
        double distanceCm = Double.isNaN(visibleDistanceCm) ? lastDistanceCm : visibleDistanceCm;
        if (Double.isNaN(distanceCm)) {
            lastShot = "NOT FIRED: no hive tag seen yet";
            return;
        }
        targetRpm = Range.clip(RPM_PER_CM * distanceCm + RPM_AT_ZERO, MIN_RPM, MAX_RPM);
        shooterMotor.setVelocity(rpmToTicksPerSec(targetRpm));
        lastShot = String.format("%.0f cm -> %.0f RPM", distanceCm, targetRpm);
        state = ShotState.SPINNING_UP;
        stateStartMs = now;
    }

    /** When at speed (or timed out), opens the gate for GATE_OPEN_MS, then closes it. */
    private void updateShot(long now) {
        if (state == ShotState.SPINNING_UP) {
            boolean atSpeed = Math.abs(actualRpm() - targetRpm) <= RPM_TOLERANCE;
            boolean timedOut = now - stateStartMs >= SPIN_UP_TIMEOUT_MS;
            if (atSpeed || timedOut) {
                gateServo.setPosition(GATE_SHOOT);
                lastShot += atSpeed ? " | fired at speed" : " | fired on timeout";
                state = ShotState.FIRING;
                stateStartMs = now;
            }
        } else if (state == ShotState.FIRING) {
            if (now - stateStartMs >= GATE_OPEN_MS) {
                gateServo.setPosition(GATE_START);
                state = ShotState.IDLE;
            }
        }
    }

    private void toggleIntake() {
        intakeOn = !intakeOn;
        intakeMotor.setPower(intakeOn ? INTAKE_POWER : 0);
        transferMotor.setPower(intakeOn ? TRANSFER_POWER : 0);
    }

    // ---- Helpers ----

    private double actualRpm() {
        return shooterMotor.getVelocity() * 60.0 / TICKS_PER_REV; // getVelocity() is ticks per second.
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    /** Mecanum drive, same math as Drivetrain.drive(). */
    private void drive(double forward, double strafe, double turn) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        leftFront.setPower((forward + strafe + turn) / denominator);
        leftBack.setPower((forward - strafe + turn) / denominator);
        rightFront.setPower((forward - strafe - turn) / denominator);
        rightBack.setPower((forward + strafe - turn) / denominator);
    }
}
