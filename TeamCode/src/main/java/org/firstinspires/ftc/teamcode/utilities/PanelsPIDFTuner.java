package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable; // Lets us edit numbers live in Panels.
import com.bylazar.telemetry.PanelsTelemetry;             // Sends numbers to Panels / Driver Station.
import com.bylazar.telemetry.TelemetryManager;            // The object we put telemetry into.
import com.qualcomm.hardware.lynx.LynxModule;              // Control / Expansion Hub.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;     // Base class for robot programs.
import com.qualcomm.robotcore.eventloop.opmode.Utility;    // Puts this OpMode under Utility on the DS.
import com.qualcomm.robotcore.hardware.DcMotor;            // Basic motor settings.
import com.qualcomm.robotcore.hardware.DcMotorEx;          // Motor that can report speed (velocity).
import com.qualcomm.robotcore.hardware.DcMotorSimple;      // FORWARD / REVERSE direction.
import com.qualcomm.robotcore.util.ElapsedTime;           // A stopwatch for timing between loops.
import com.qualcomm.robotcore.util.Range;                 // Range.clip keeps a number in a min..max range.

/*
 * ============================================================================
 *  PANELS PIDF TUNER  (simple version)
 * ============================================================================
 *
 * WHAT THIS DOES
 *   Spins the shooter motor and tries to hold a target speed (RPM).
 *   You change P, I, D, F, TARGET_RPM, and MOTOR_REVERSED in Panels until the wheel
 *   reaches the target fast and stays there.
 *
 * HOW TO USE
 *   1. Connect to the robot Wi-Fi and open Panels.
 *   2. On the Driver Station: Utility > "Panels PIDF Tuner" > INIT > START.
 *      On START the motor immediately spins toward TARGET_RPM (using F as the guess).
 *   3. In Panels > Configurables, edit P / I / D / F / TARGET_RPM / MOTOR_REVERSED.
 *   4. After typing a number, press ENTER (or "Update all"). Typing alone
 *      does NOT send the new value to the robot.
 *
 * WHAT THE LETTERS MEAN  (think: cruise control in a car)
 *   TARGET_RPM = speed you want                  (set cruise to 60 mph)
 *   Current RPM = speed you actually have        (speedometer says 55)
 *   Error       = target - current               (5 mph too slow)
 *
 *   F = "the guess"     Power you already expect to need for that speed.
 *   P = "fix it now"    Bigger error -> push harder right away.
 *   I = "fix it slowly" Small leftover error that won't go away -> keep adding a little.
 *   D = "the brakes"    Closing in too fast -> ease off so you don't overshoot.
 *
 *   Motor power = F part + P part + I part + D part  (then clipped to -1..1)
 *
 * WHEN YOU'RE DONE
 *   Copy P, I, D, F into Shooter.java. Panels values reset when the robot restarts.
 * ============================================================================
 */
@Configurable // Every public static field below shows up (and is editable) in Panels.
@Utility(name = "Panels PIDF Tuner")
public class PanelsPIDFTuner extends OpMode {

    // ======================= EDIT THESE IN PANELS =======================
    // Press ENTER after each change so the robot actually gets the new value.
    public static double P = 0.005;           // Push harder when far from target. Too big = wobbles.
    public static double I = 0.0;           // Slowly removes small leftover error.
    public static double D = 0.0;           // Slows down when closing in fast (stops overshoot).
    // Base power guess: ~full power at 6000 RPM. Without F (and with P=0) the motor never moves.
    public static double F = 0.00025;
    public static double TARGET_RPM = 2050; // Speed we want the shooter wheel to spin at.
    // Boolean (not DcMotorSimple.Direction): SDK enums can break Panels live updates for the whole class.
    public static boolean MOTOR_REVERSED = false;

    // ======================= FIXED SETTINGS (not in Panels) =======================
    // Must match the motor name in the robot configuration on the Driver Station.
    private static final String MOTOR_NAME = "ShooterMotor";
    // Encoder ticks for one full turn of a goBILDA 6000 RPM bare motor.
    private static final double TICKS_PER_REV = 28.0;
    // Motor power can never go outside -1.0 .. 1.0.
    private static final double MAX_POWER = 1.0;
    // Cap on how much power the I term is allowed to add (stops "integral windup").
    private static final double MAX_INTEGRAL_POWER = 0.25;
    // Smooths the D term so noisy encoder data doesn't make the motor jitter. 0 = off, ~0.9 = very smooth.
    private static final double D_FILTER = 0.7;

    // ======================= INTERNAL STATE =======================
    private TelemetryManager panels; // Where we send numbers for the DS / Panels.
    private DcMotorEx motor;         // The shooter motor (null if the name was wrong).
    private double lastAppliedPower = 0; // Last power we sent (for telemetry).

    // PIDF memory that carries from one loop to the next.
    private double integralSum = 0;          // Running total of error over time (used by I).
    private double lastError = 0;            // Error from the previous loop (used by D).
    private double filteredDerivative = 0;   // Smoothed "how fast is error changing?" value.
    private final ElapsedTime pidTimer = new ElapsedTime(); // Times how long since last PID update.

    // Runs once when you press INIT on the Driver Station.
    @Override
    public void init() {
        // Get the Panels telemetry helper so numbers show on the laptop and DS.
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // Make hub sensor reads faster (one bulk read per loop instead of many slow ones).
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Find the motor by name. tryGet returns null instead of crashing if the name is wrong.
        motor = hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAME);
        if (motor != null) {
            applyDirection(); // Respect MOTOR_REVERSED before anything else.
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Start encoder count at 0.
            // We calculate PIDF ourselves, so the hub's built-in speed control stays off.
            // The encoder still measures speed — we just send raw power with setPower().
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Flywheel: coast when power is 0 (don't slam on the brakes).
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPower(0); // Stay still during INIT (safe).
        }
    }

    // Runs over and over after INIT, before you press START. Shows telemetry only.
    @Override
    public void init_loop() {
        applyDirection(); // Let MOTOR_REVERSED changes from Panels take effect before START.
        sendTelemetry(readRpm());
    }

    // Runs once when you press START / PLAY — immediately chase TARGET_RPM.
    @Override
    public void start() {
        // Clear old I / D memory so a previous run doesn't mess up this one.
        resetController();
        if (motor == null) return;

        applyDirection();
        // Kick the motor right away (don't wait for the first loop).
        // With default F this is already a non-zero power toward TARGET_RPM.
        lastAppliedPower = calculatePIDF(TARGET_RPM, readRpm());
        motor.setPower(lastAppliedPower);
    }

    // Runs over and over while the OpMode is playing (~50 times per second).
    @Override
    public void loop() {
        // Safety: if we never found the motor, just show that and do nothing.
        if (motor == null) {
            sendTelemetry(0);
            return;
        }

        applyDirection(); // Live Panels updates for MOTOR_REVERSED.

        // 1) Measure how fast we are going right now.
        double currentRpm = readRpm();
        // 2) Figure out how much power to send (PIDF).
        // 3) Send that power to the motor.
        lastAppliedPower = calculatePIDF(TARGET_RPM, currentRpm);
        motor.setPower(lastAppliedPower);
        // 4) Show the numbers on the Driver Station / Panels.
        sendTelemetry(currentRpm);
    }

    // Runs once when you press STOP.
    @Override
    public void stop() {
        if (motor != null) motor.setPower(0); // Always turn the motor off when we stop.
        lastAppliedPower = 0;
    }

    // Apply the Panels MOTOR_REVERSED setting. Safe to call every loop.
    private void applyDirection() {
        if (motor == null) return;
        motor.setDirection(MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
    }

    // ============================================================================
    //  PIDF MATH
    //  Returns motor power from -MAX_POWER to +MAX_POWER.
    // ============================================================================
    private double calculatePIDF(double target, double measured) {
        // dt = seconds since the last time we ran this math.
        double dt = pidTimer.seconds();
        pidTimer.reset();
        // First loop (or a long pause) can give a weird dt — use a normal ~50 Hz value instead.
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        // How far off are we? Positive = too slow, negative = too fast.
        double error = target - measured;

        // ---- P term: fix it now ----
        // Farther from target = stronger push. Example: P=0.001, error=200 RPM -> 0.2 power.
        double pOut = P * error;

        // ---- I term: fix it slowly ----
        // Add up error over time. If we're always a little low, this grows and pushes harder.
        integralSum += error * dt;
        if (I != 0) {
            // Don't let I grow forever (that can make the motor slam too hard later).
            double limit = MAX_INTEGRAL_POWER / Math.abs(I);
            integralSum = Range.clip(integralSum, -limit, limit);
        } else {
            // I is off — keep the sum at 0 so it doesn't surprise us later.
            integralSum = 0;
        }
        double iOut = I * integralSum;

        // ---- D term: the brakes ----
        // How fast is the error changing? If we're rushing toward the target, ease off.
        double rawDerivative = (error - lastError) / dt;
        // Blend old + new so encoder noise doesn't make D jump around.
        filteredDerivative = D_FILTER * filteredDerivative + (1 - D_FILTER) * rawDerivative;
        double dOut = D * filteredDerivative;
        lastError = error; // Remember for next loop.

        // ---- F term: the guess ----
        // For velocity: power we expect for this target RPM (same idea as Shooter kF).
        // Example: F = 1/6000 means full power (~1.0) at 6000 RPM with P=I=D=0.
        double fOut = F * target;

        // Add all four parts, then clamp so we never ask for more than ±100% power.
        return Range.clip(pOut + iOut + dOut + fOut, -MAX_POWER, MAX_POWER);
    }

    // Wipe the PIDF memory clean (call this when starting a fresh run).
    private void resetController() {
        integralSum = 0;
        lastError = 0;
        filteredDerivative = 0;
        pidTimer.reset();
    }

    // Reads motor speed and converts encoder ticks/sec into RPM (revolutions per minute).
    private double readRpm() {
        if (motor == null || TICKS_PER_REV <= 0) return 0;
        // getVelocity() = ticks per second. Divide by ticks per turn, then * 60 for per-minute.
        return motor.getVelocity() / TICKS_PER_REV * 60.0;
    }

    // Shows the important numbers on the Driver Station and in Panels.
    private void sendTelemetry(double currentRpm) {
        if (motor == null) {
            panels.addLine("Motor not found: \"" + MOTOR_NAME + "\"");
            panels.update(telemetry);
            return;
        }

        // Current = how fast we are. Target = how fast we want. Error = the difference.
        panels.addData("Current RPM", currentRpm);
        panels.addData("Target RPM", TARGET_RPM);
        panels.addData("Error", TARGET_RPM - currentRpm);
        panels.addData("Power", lastAppliedPower);
        panels.addData("Motor reversed", MOTOR_REVERSED);
        // The four gains you're tuning (should match what you just set in Panels).
        panels.addData("P", P);
        panels.addData("I", I);
        panels.addData("D", D);
        panels.addData("F", F);

        // Push everything to the Driver Station screen (and Panels graphs / telemetry).
        panels.update(telemetry);
    }
}
