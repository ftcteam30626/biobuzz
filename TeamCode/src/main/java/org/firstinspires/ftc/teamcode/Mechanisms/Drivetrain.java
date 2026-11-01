package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;  // Lets Panels edit the settings live.
import com.pedropathing.ivy.Command;                        // A small action the robot can run.
import com.qualcomm.robotcore.hardware.Gamepad;             // The controller the driver holds.
import com.qualcomm.robotcore.util.ElapsedTime;             // A stopwatch.
import com.qualcomm.robotcore.util.Range;                   // Range.clip(x, min, max) keeps x between min and max.

import dev.nextftc.hardware.actuators.NextMotor;            // NextFTC's motor.
import dev.nextftc.robot.Mechanism;                         // Marks this class as one part of the robot.
import dev.nextftc.robot.Telemetry;                         // Sends text/numbers to the Driver Station screen.
import dev.nextftc.units.Units;                             // Unit helpers (used to read encoder ticks).

/*
 * DRIVETRAIN MECHANISM (smooth mecanum drive)
 *
 * HARDWARE (names must match the robot configuration on the Driver Station):
 *   leftFront, rightFront, rightBack, leftBack = the 4 drive motors
 *
 * CONTROLS (whichever gamepad is passed to teleopDrive, usually gamepad 1):
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
public class Drivetrain implements Mechanism {

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
    // NextMotor doesn't touch the hardwareMap until it's first used, so it's safe to create them here.
    private final NextMotor leftFront = new NextMotor("leftFront");
    private final NextMotor rightFront = new NextMotor("rightFront");
    private final NextMotor rightBack = new NextMotor("rightBack");
    private final NextMotor leftBack = new NextMotor("leftBack");
    // Same 4 motors in a list, so we can loop over them. This order is used everywhere below.
    private final NextMotor[] motors = {leftFront, rightFront, rightBack, leftBack};

    // ---- State ----
    // Each array has one slot per wheel, in this order: leftFront, rightFront, rightBack, leftBack.
    private final double[] currentPower = new double[4]; // Power each wheel has right now (after ramping).
    private final double[] holdTarget = new double[4];   // Encoder spot (ticks) each wheel is holding.
    private boolean holding = false;                     // true while the wheels are holding position.
    private String mode = "STOPPED";                     // Shown on telemetry.

    private final ElapsedTime loopTimer = new ElapsedTime(); // Measures time between loops.

    public Drivetrain() {
        // BRAKE = the motor resists turning at 0 power, so the robot stops hard.
        for (NextMotor motor : motors) motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    // ---- Commands ----

    /**
     * Drives with the sticks of the given gamepad, forever (until the OpMode stops).
     * infinite(...) makes a Command that runs the code inside it every loop.
     */
    public Command teleopDrive(Gamepad gamepad) {
        return infinite(() -> {
            // Stick up is negative on the gamepad, so flip Y.
            double drive = shape(-gamepad.left_stick_y);
            double strafe = shape(gamepad.left_stick_x) * STRAFE_FIX;
            double turn = shape(gamepad.right_stick_x);
            drive(drive, strafe, turn);
        });
    }

    /**
     * Stops all 4 wheels right now (not as a command). Use this in an OpMode's end(),
     * because commands don't get a chance to run after STOP is pressed.
     */
    public void stopNow() {
        holding = false;
        mode = "STOPPED";
        for (int i = 0; i < 4; i++) {
            currentPower[i] = 0;
            motors[i].setThrottle(0.0);
            motors[i].update(); // Send the 0 power to the motor immediately.
        }
    }

    // ---- Driving ----

    /** Drives with already-shaped values from -1 to 1 (drive = forward, strafe = right, turn = clockwise). */
    public void drive(double drive, double strafe, double turn) {
        // dt = seconds since the last loop. Used so the ramp is the same no matter how fast we loop.
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0.1) dt = 0.02; // First loop or a long pause: use a normal value.
        applyDirections(); // Re-apply in case they were changed in Panels.

        if (drive == 0 && strafe == 0 && turn == 0) {
            brake();
        } else {
            holding = false;
            mode = "DRIVING";
            driveSmooth(drive, strafe, turn, dt);
        }
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
        double maxPower = Range.clip(MAX_POWER, 0, 1); // NextMotor crashes on power above 1.
        double biggest = 1.0;
        for (double p : target) biggest = Math.max(biggest, Math.abs(p));
        for (int i = 0; i < 4; i++) {
            target[i] = target[i] / biggest * maxPower;

            // Speeding up (or reversing) = slow ramp. Slowing down = fast ramp.
            boolean speedingUp = Math.abs(target[i]) > Math.abs(currentPower[i])
                    || Math.signum(target[i]) != Math.signum(currentPower[i]);
            double maxStep = (speedingUp ? ACCEL_PER_SEC : DECEL_PER_SEC) * dt; // Most power can change this loop.
            currentPower[i] += Range.clip(target[i] - currentPower[i], -maxStep, maxStep);

            motors[i].setThrottle(currentPower[i]);
        }
    }

    // Stops the robot, then (optionally) holds every wheel where it stopped.
    private void brake() {
        // Cut power instantly. BRAKE mode shorts the motor so the wheels stop hard.
        for (int i = 0; i < 4; i++) currentPower[i] = 0;

        if (!HOLD_POSITION) {
            holding = false;
            mode = "BRAKING";
            for (NextMotor motor : motors) motor.setThrottle(0.0);
            return;
        }

        // Lock in where the wheels are the moment the sticks are let go.
        if (!holding) {
            for (int i = 0; i < 4; i++) holdTarget[i] = ticks(motors[i]);
            holding = true;
            mode = "HOLDING";
        }

        // Push each wheel back toward its locked spot (a simple P controller).
        for (int i = 0; i < 4; i++) {
            double error = holdTarget[i] - ticks(motors[i]); // How far it got pushed.
            double power = Math.abs(error) <= HOLD_TOLERANCE ? 0
                    : Range.clip(error * HOLD_P, -HOLD_MAX_POWER, HOLD_MAX_POWER);
            motors[i].setThrottle(power);
        }
    }

    // ---- Helpers ----

    // Deadzone + curve, keeping the sign. Rescaled so just past the deadzone starts at 0.
    // Example with CURVE = 2: half stick (0.5) gives about 0.22 power instead of 0.5.
    private static double shape(double input) {
        double magnitude = Math.abs(input);
        if (magnitude < DEADZONE) return 0;
        double scaled = (magnitude - DEADZONE) / (1 - DEADZONE);
        return Math.signum(input) * Math.pow(scaled, CURVE); // signum = +1 or -1, keeps the direction.
    }

    // A motor's encoder position in ticks. NextMotor counts 1 tick as "1 radian" by default,
    // so reading it in radians gives back the raw tick count.
    private static double ticks(NextMotor motor) {
        return motor.getEncoderPosition().into(Units.Radians);
    }

    // Sets each motor's direction from the *_REVERSED settings.
    private void applyDirections() {
        leftFront.setDirection(dir(LEFT_FRONT_REVERSED));
        rightFront.setDirection(dir(RIGHT_FRONT_REVERSED));
        rightBack.setDirection(dir(RIGHT_BACK_REVERSED));
        leftBack.setDirection(dir(LEFT_BACK_REVERSED));
    }

    // Turns true/false into REVERSE/FORWARD.
    private static NextMotor.Direction dir(boolean reversed) {
        return reversed ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD;
    }

    // Runs every loop automatically, so we use it to show what the drivetrain is doing.
    @Override
    public void periodic() {
        Telemetry.log("Drive mode", mode);
        Telemetry.log("LF / RF", String.format("%.2f / %.2f", currentPower[0], currentPower[1]));
        Telemetry.log("LB / RB", String.format("%.2f / %.2f", currentPower[3], currentPower[2]));
    }
}
