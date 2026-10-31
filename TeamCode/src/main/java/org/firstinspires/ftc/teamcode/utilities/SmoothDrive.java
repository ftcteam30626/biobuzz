package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;     // Lets Panels edit the settings live.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;         // Basic FTC OpMode (init/loop/stop).
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;         // Puts this OpMode in the TeleOp list.
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;                // Motor settings (BRAKE, run modes).
import com.qualcomm.robotcore.hardware.DcMotorEx;              // Motor with extra features (encoder, velocity).
import com.qualcomm.robotcore.hardware.DcMotorSimple;          // Motor direction (FORWARD / REVERSE).
import com.qualcomm.robotcore.util.ElapsedTime;                // A stopwatch.
import com.qualcomm.robotcore.util.Range;                      // Range.clip(x, min, max) keeps x between min and max.

/*
 * SMOOTH MECANUM DRIVE
 *
 * CONTROLS (gamepad 1):
 *   Left stick  = drive (up/down = forward/back, left/right = strafe)
 *   Right stick = turn (left/right)
 *
 * HOW IT STAYS SMOOTH:
 *   1. Deadzone: tiny stick wiggles are ignored.
 *   2. Curve: small stick moves give extra-small power, so slow driving is easy to control.
 *   3. Ramp: power can only go UP by a limited amount each second, so the robot doesn't jerk.
 *
 * HOW IT BRAKES:
 *   1. Motors are set to BRAKE, and power drops to 0 instantly (no ramp when slowing down to 0).
 *   2. Once stopped, every wheel holds its position with a small P controller,
 *      so the robot fights back if it gets pushed or keeps sliding.
 *
 * If the robot drives the wrong way, flip the *_REVERSED settings (live in Panels).
 */
@Configurable
@Utility(name = "Smooth Drive")
public class SmoothDrive extends OpMode {

    // ---- Settings you can change live in Panels ----
    // Which wheels spin backwards. One side of a robot is usually flipped.
    public static boolean LEFT_FRONT_REVERSED = true;
    public static boolean LEFT_BACK_REVERSED = true;
    public static boolean RIGHT_FRONT_REVERSED = false;
    public static boolean RIGHT_BACK_REVERSED = false;

    public static double DEADZONE = 0.05;      // Stick moves smaller than this count as 0.
    public static double CURVE = 2.0;          // 1 = linear, 2 = squared, 3 = cubed. Higher = finer slow control.
    public static double ACCEL_PER_SEC = 4.0;  // How fast power can rise (4 = 0 to full in 0.25 s).
    public static double DECEL_PER_SEC = 12.0; // How fast power can drop while still moving (much faster than accel).
    public static double STRAFE_FIX = 1.1;     // Mecanum strafes slower than it drives; this evens it out.
    public static double MAX_POWER = 1.0;      // Top speed (lower it to make the robot slower).

    public static boolean HOLD_POSITION = true; // true = wheels fight back when pushed while stopped.
    public static double HOLD_P = 0.01;         // Power per encoder tick of error while holding.
    public static double HOLD_MAX_POWER = 0.3;  // Most power the hold is allowed to use.
    public static int HOLD_TOLERANCE = 5;       // Ticks of error to ignore (stops buzzing).

    // ---- Hardware ----
    private DcMotorEx leftFront, rightFront, rightBack, leftBack;
    private DcMotorEx[] motors; // Same 4 motors in a list, so we can loop over them.

    // ---- State ----
    // Each array has one slot per wheel, in this order: leftFront, rightFront, rightBack, leftBack.
    private final double[] currentPower = new double[4]; // Power each wheel has right now (after ramping).
    private final int[] holdTarget = new int[4];         // Encoder spot each wheel is holding.
    private boolean holding = false;                     // true while the wheels are holding position.

    private final ElapsedTime loopTimer = new ElapsedTime(); // Measures time between loops.

    // Runs once when INIT is pressed.
    @Override
    public void init() {
        // Find each motor by its name in the robot configuration.
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        motors = new DcMotorEx[]{leftFront, rightFront, rightBack, leftBack};

        applyDirections();
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Stop hard at 0 power.
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        // Set encoder to 0.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);           // We control power ourselves.
        }

        telemetry.addLine("Ready. Left stick drives, right stick turns.");
    }

    // Runs once when PLAY is pressed.
    @Override
    public void start() {
        loopTimer.reset();
    }

    // Runs over and over while the OpMode is playing.
    @Override
    public void loop() {
        // dt = seconds since the last loop. Used so the ramp is the same no matter how fast we loop.
        double dt = loopTimer.seconds();
        loopTimer.reset();
        applyDirections(); // Re-apply in case they were changed in Panels.

        // Stick up is negative on the gamepad, so flip Y.
        double drive = shape(-gamepad1.left_stick_y);
        double strafe = shape(gamepad1.left_stick_x) * STRAFE_FIX;
        double turn = shape(gamepad1.right_stick_x);

        boolean sticksIdle = drive == 0 && strafe == 0 && turn == 0;

        if (sticksIdle) {
            brake();
        } else {
            holding = false;
            driveSmooth(drive, strafe, turn, dt);
        }

        // Show what's happening on the Driver Station.
        telemetry.addData("Mode", holding ? "HOLDING" : (sticksIdle ? "BRAKING" : "DRIVING"));
        telemetry.addData("Drive / Strafe / Turn", "%.2f / %.2f / %.2f", drive, strafe, turn);
        telemetry.addData("LF / RF", "%.2f / %.2f", currentPower[0], currentPower[1]);
        telemetry.addData("LB / RB", "%.2f / %.2f", currentPower[3], currentPower[2]);
    }

    // Runs once when STOP is pressed.
    @Override
    public void stop() {
        for (DcMotorEx motor : motors) motor.setPower(0);
    }

    // Works out each wheel's power from the sticks, then ramps toward it.
    private void driveSmooth(double drive, double strafe, double turn, double dt) {
        // Standard mecanum math: each wheel mixes drive, strafe and turn differently.
        double[] target = {
                drive + strafe + turn, // leftFront
                drive - strafe - turn, // rightFront
                drive + strafe - turn, // rightBack
                drive - strafe + turn  // leftBack
        };

        // Scale down so no wheel goes over MAX_POWER (keeps the robot driving straight).
        double biggest = 1.0;
        for (double p : target) biggest = Math.max(biggest, Math.abs(p));
        for (int i = 0; i < 4; i++) {
            target[i] = target[i] / biggest * MAX_POWER;

            // Speeding up (or reversing) = slow ramp. Slowing down = fast ramp.
            boolean speedingUp = Math.abs(target[i]) > Math.abs(currentPower[i])
                    || Math.signum(target[i]) != Math.signum(currentPower[i]);
            double maxStep = (speedingUp ? ACCEL_PER_SEC : DECEL_PER_SEC) * dt; // Most power can change this loop.
            currentPower[i] += Range.clip(target[i] - currentPower[i], -maxStep, maxStep);

            motors[i].setPower(currentPower[i]);
        }
    }

    // Stops the robot, then (optionally) holds every wheel where it stopped.
    private void brake() {
        // Cut power instantly. BRAKE mode shorts the motor so the wheels stop hard.
        for (int i = 0; i < 4; i++) currentPower[i] = 0;

        if (!HOLD_POSITION) {
            holding = false;
            for (DcMotorEx motor : motors) motor.setPower(0);
            return;
        }

        // Lock in where the wheels are the moment the sticks are let go.
        if (!holding) {
            for (int i = 0; i < 4; i++) holdTarget[i] = motors[i].getCurrentPosition();
            holding = true;
        }

        // Push each wheel back toward its locked spot (a simple P controller).
        for (int i = 0; i < 4; i++) {
            int error = holdTarget[i] - motors[i].getCurrentPosition(); // How far it got pushed.
            double power = Math.abs(error) <= HOLD_TOLERANCE ? 0
                    : Range.clip(error * HOLD_P, -HOLD_MAX_POWER, HOLD_MAX_POWER);
            motors[i].setPower(power);
        }
    }

    // Deadzone + curve, keeping the sign. Rescaled so just past the deadzone starts at 0.
    // Example with CURVE = 2: half stick (0.5) gives about 0.22 power instead of 0.5.
    private double shape(double input) {
        double magnitude = Math.abs(input);
        if (magnitude < DEADZONE) return 0;
        double scaled = (magnitude - DEADZONE) / (1 - DEADZONE);
        return Math.signum(input) * Math.pow(scaled, CURVE); // signum = +1 or -1, keeps the direction.
    }

    // Sets each motor's direction from the *_REVERSED settings.
    private void applyDirections() {
        leftFront.setDirection(dir(LEFT_FRONT_REVERSED));
        rightFront.setDirection(dir(RIGHT_FRONT_REVERSED));
        rightBack.setDirection(dir(RIGHT_BACK_REVERSED));
        leftBack.setDirection(dir(LEFT_BACK_REVERSED));
    }

    // Turns true/false into REVERSE/FORWARD.
    private static DcMotorSimple.Direction dir(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }
}
