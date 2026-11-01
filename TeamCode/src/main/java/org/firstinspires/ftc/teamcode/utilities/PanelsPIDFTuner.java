package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;  // Lets Panels edit the settings live.
import com.bylazar.telemetry.PanelsTelemetry;              // Sends data to Panels (graphs).
import com.bylazar.telemetry.TelemetryManager;             // The object we add Panels data to.
import com.qualcomm.hardware.lynx.LynxModule;               // The Control/Expansion Hub (for fast reads).
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;    // P, I, D, F packed together for the hub.
import com.qualcomm.robotcore.hardware.VoltageSensor;       // Reads the battery voltage.
import com.qualcomm.robotcore.util.ElapsedTime;            // A stopwatch.
import com.qualcomm.robotcore.util.Range;                  // Range.clip(x, min, max) keeps x in range.

/*
 * ============================================================================
 *  PANELS PIDF TUNER  -  tune the PIDF of ANY motor, live, from Panels
 * ============================================================================
 *
 * WHAT IS THIS FOR?
 *   You want a motor to hit a number and stay there. Examples:
 *     - a shooter flywheel that should spin at exactly 4000 RPM, even after a shot
 *     - a lift that should go to height 800 ticks and stop, without bouncing
 *   "Just set the power" doesn't work: the battery drains, things rub, balls slow the
 *   wheel down. A PIDF controller fixes this by checking the motor ~50 times a second
 *   and changing the power to keep it on target. This OpMode helps you pick the 4
 *   numbers (P, I, D, F) that make it work well, while you watch a live graph.
 *   VELOCITY mode uses RPM end-to-end so gains drop straight into Shooter-style code.
 *
 * ----------------------------------------------------------------------------
 * PIDF IN PLAIN WORDS  (think: cruise control in a car)
 * ----------------------------------------------------------------------------
 *   target   = where you WANT to be       (cruise control set to 60 mph)
 *   measured = where you ACTUALLY are     (the speedometer says 55 mph)
 *   error    = target - measured          (5 mph too slow)
 *
 *   The motor's power is made of 4 parts added together:
 *
 *   F  "the guess"     Power you already know you'll need, before looking at the error.
 *                      "60 mph usually takes about this much gas." Does most of the work.
 *   P  "fix it now"    Bigger error = push harder. 5 mph slow -> a bit more gas.
 *                      Too much P = the car lurches back and forth around 60.
 *   I  "fix it slowly" If you're STILL a little off after a while, keep adding a little
 *                      more push until it's gone. (Like going uphill.) Too much I = wobbly.
 *   D  "the brakes"    If you're closing in fast, ease off so you don't fly past the target.
 *
 *   power = F part + P part + I part + D part   (kS is an extra "unstick" push, see below)
 *
 * ----------------------------------------------------------------------------
 * WORDS YOU'LL SEE
 * ----------------------------------------------------------------------------
 *   RPM          Revolutions per minute. VELOCITY targets, measured speed, and CUSTOM_CODE
 *                gains all use RPM (matches NextFTC / Shooter-style code).
 *   ticks        Encoder counts. POSITION mode still uses ticks for target/measured.
 *   VELOCITY     Control SPEED. For flywheels, shooters, intakes. Units = RPM.
 *   POSITION     Control WHERE it is. For arms, lifts, turrets. Units = ticks.
 *   kS           Tiny push that's always added to get past friction (the motor "sticking").
 *   rise time    How long it took to get 90% of the way to the target. Smaller = faster.
 *   overshoot    How far it went PAST the target, in %. Want close to 0%.
 *   settle time  How long until it got close (within TOLERANCE) and stayed there.
 *   steady err   How far off it sits once settled. Want close to 0.
 *
 * ----------------------------------------------------------------------------
 * READING THE GRAPH  (plot "target" and "measured")
 * ----------------------------------------------------------------------------
 *   What you see                                   What to do
 *   measured is way off / never gets there      -> F too small (or REVERSED is wrong)
 *   measured slowly creeps up to target         -> raise P
 *   measured wiggles / buzzes around target     -> lower P (and I)
 *   measured shoots PAST target, then comes back-> lower P, or add a little D
 *   measured stops a little short, forever      -> add a tiny I (or raise F)
 *   measured jumps up fast and stays flat on it -> DONE!
 *
 * ----------------------------------------------------------------------------
 * SETUP (once)
 * ----------------------------------------------------------------------------
 *   1. Connect the laptop to the robot's Wi-Fi.
 *   2. Open Panels in a browser: http://192.168.43.1:8001
 *   3. On the Driver Station pick Utility "Panels PIDF Tuner" and press INIT.
 *   4. In Panels > Configurables > PanelsPIDFTuner set:
 *        MOTOR_NAME     = robot config name (default ShooterMotor — change if needed)
 *        MOTOR_TYPE     = your motor (or CUSTOM and fill CUSTOM_TICKS_PER_REV)
 *                         (needed so RPM <-> encoder conversion is correct)
 *        CONTROL_MODE   = VELOCITY (RPM) or POSITION (ticks)
 *        CONTROLLER     = leave on CUSTOM_CODE (see GAIN UNITS at the bottom)
 *        VOLTAGE_COMPENSATION = leave OFF for Shooter.java (it has no battery scale)
 *      Telemetry says "Motor: OK" when it found the motor.
 *   5. In Panels > Graph, add: target, measured (and error / power if you want).
 *   6. Press START.
 *
 * GAMEPAD 1 CONTROLS (after START)
 *   A = controller ON        B = controller OFF (motor stops)
 *   X = go to TARGET_1       Y = go to TARGET_2
 *   Dpad up / down  = nudge the current target by NUDGE
 *   Right bumper    = run the F finder (VELOCITY mode only, motor will spin at FULL power!)
 *   Back            = reset the step-test stats
 *   (Every button also has a Panels switch: ENABLED, RUN_F_FINDER, TEST_MODE ...)
 *
 * ----------------------------------------------------------------------------
 * TUNING STEP BY STEP
 * ----------------------------------------------------------------------------
 *   VELOCITY (flywheel / shooter):
 *     1. Set P, I, D to 0.
 *     2. Press right bumper (F finder). The motor spins at full power for a few seconds,
 *        averages steady-state RPM (not peak), and fills in F for you. Keep hands clear!
 *     3. Press A. With only F, "measured" should end up near "target" (a bit low is fine).
 *     4. Set TEST_MODE = STEP so the target flips on its own. Now raise P a little at a
 *        time (try 0.0005, 0.001, 0.002 ...). Stop when it gets to target quickly.
 *        If it starts to wiggle or buzz, go back down by about a third.
 *     5. Still sitting a bit under target? Add a tiny I (about P / 10).
 *     6. Leave D at 0. Flywheels almost never need it.
 *
 *   POSITION (arm / lift):
 *     1. Set everything to 0.
 *     2. F: for a lift, slowly raise F until the lift just barely holds still on its own
 *        (fights gravity). For a flat turret, leave F = 0.
 *     3. kS: raise it until the motor just barely starts to move. Then back off a tiny bit.
 *     4. Press A, set TEST_MODE = STEP. Raise P until it reaches the target quickly,
 *        even if it goes a little past.
 *     5. Raise D until it stops going past / bouncing.
 *     6. Stops a little short of the target? Add a tiny I.
 *
 *   Each time the target changes, telemetry shows the STEP line:
 *     "rise 0.30s | overshoot 2.0% | settle 0.45s | steady err 3.1"
 *   Try to make all of those numbers small.
 *
 * WHEN YOU'RE DONE
 *   Panels changes are LOST when the robot restarts! Copy the "COPY THIS" line from
 *   telemetry into your real code.
 *
 * GAIN UNITS (why the numbers look so tiny)
 *   CUSTOM_CODE + VELOCITY: error is in RPM, power is -1..1. So P is tiny:
 *     P = 0.0005 with 200 RPM too slow -> 0.1 extra power. F ~= 1 / free-speed RPM
 *     (e.g. 1/6000 for a bare 6000 RPM motor). Same units as Shooter.java.
 *   CUSTOM_CODE + POSITION: error is in ticks; P is still tiny for the same reason.
 *   HUB_BUILT_IN: targets are still RPM (converted for you), but P/I/D/F use hub units
 *     (roughly F = 32767 / max ticks/sec). Do NOT mix hub gains into CUSTOM_CODE.
 *
 * SHOOTER / NextFTC TRANSFER
 *   Defaults match Shooter.java: VOLTAGE_COMPENSATION off, MOTOR_NAME = ShooterMotor.
 *   Leave voltage compensation OFF unless your competition code also scales by battery.
 *   Shooter currently uses kP/kI/kD/kV only (no kS) — keep kS at 0 when copying there.
 * ============================================================================
 */
@Configurable // Makes every public static field below show up (and be editable) in Panels.
@Utility(name = "Panels PIDF Tuner")
public class PanelsPIDFTuner extends OpMode {

    // ======================= INSTRUCTIONS (shown in Panels) =======================
    // These are only here so the steps show up inside Panels > Configurables.
    // Changing them does nothing.
    public static String HOW_TO_0 = "Goal: make the 'measured' line jump to the 'target' line fast and stay flat on it";
    public static String HOW_TO_1 = "Type MOTOR_NAME, pick MOTOR_TYPE (for RPM math) + CONTROL_MODE (VELOCITY=RPM, POSITION=ticks)";
    public static String HOW_TO_2 = "Graph tab: add 'target' and 'measured'. Press START, then A on gamepad1 to turn on";
    public static String HOW_TO_3 = "F = base power guess | P = push harder when far off | I = fix small leftover error | D = brakes";
    public static String HOW_TO_4 = "VELOCITY (RPM): P=I=D=0, right bumper finds F, then raise P slowly. Wiggles? lower P";
    public static String HOW_TO_5 = "POSITION (ticks): F = hold power, kS = just-moves power, raise P, add D if it overshoots";
    public static String HOW_TO_6 = "TEST_MODE = STEP flips targets by itself. Try to make rise/overshoot/settle small";
    public static String HOW_TO_7 = "Copy 'COPY THIS (Shooter)' into Shooter.java. Keep VOLTAGE_COMPENSATION off + kS=0 to match. Values reset on restart!";

    // ======================= MOTOR =======================
    public static String MOTOR_NAME = "ShooterMotor"; // Must match the robot configuration name.
    public static boolean REVERSED = false;    // true = spin the other way.
    public static MotorType MOTOR_TYPE = MotorType.GOBILDA_6000_RPM_BARE;
    public static double CUSTOM_TICKS_PER_REV = 28; // only used when MOTOR_TYPE = CUSTOM

    // ======================= MODE =======================
    public static ControlMode CONTROL_MODE = ControlMode.VELOCITY; // Control speed or position?
    public static Controller CONTROLLER = Controller.CUSTOM_CODE;  // Our PIDF or the hub's?
    public static TestMode TEST_MODE = TestMode.HOLD;              // Stay on one target or flip?
    public static boolean ENABLED = false;     // same as A / B on the gamepad

    // ======================= GAINS =======================
    public static double P = 0.0; // "Fix it now": further off = push harder. Too big = wiggles.
    public static double I = 0.0; // "Fix it slowly": removes small error that won't go away.
    public static double D = 0.0; // "Brakes": eases off when closing in fast. Stops overshoot.
    public static double F = 0.0; // "The guess": power we already know we need. Start here.
    public static double kS = 0.0;              // CUSTOM_CODE only; Shooter.java currently ignores this

    // ======================= TARGETS =======================
    // VELOCITY = RPM, POSITION = encoder ticks.
    public static double TARGET_1 = 4000;
    public static double TARGET_2 = 0;
    public static double STEP_TIME = 2.0;       // seconds between flips in STEP mode
    public static double NUDGE = 100;           // dpad up/down: RPM in VELOCITY, ticks in POSITION

    // ======================= SAFETY / EXTRAS (CUSTOM_CODE only) =======================
    public static double MAX_POWER = 1.0;            // Most power the controller can send.
    public static double MAX_INTEGRAL_POWER = 0.25; // most power the I term may add
    public static double D_FILTER = 0.7;         // Smooths D so it doesn't jitter. 0 = off, 0.9 = very smooth
    // OFF by default: Shooter / NextFTC velocity mode does not scale by battery. Turn on only if
    // competition code also multiplies output by NOMINAL_VOLTAGE / battery.
    public static boolean VOLTAGE_COMPENSATION = false;
    public static double NOMINAL_VOLTAGE = 12.0;       // Voltage the gains were tuned at (only if compensation on).

    // ======================= STATS =======================
    public static double TOLERANCE = 50;         // "close enough" (RPM in VELOCITY, ticks in POSITION)
    public static double SETTLE_TIME = 0.3;      // must stay in tolerance this long to count as settled

    // ======================= F FINDER =======================
    public static boolean RUN_F_FINDER = false;  // set true in Panels (or right bumper) to run it
    public static double F_FINDER_SECONDS = 2.5;     // How long the F finder spins at full power.
    public static boolean F_FINDER_AUTO_APPLY = true; // put the found F straight into F (does NOT change TARGET_1)

    // ======================= TELEMETRY =======================
    public static boolean SHOW_HELP = true;       // show the how-to lines in telemetry

    // An enum is a fixed list of choices. Panels shows these as a dropdown.
    public enum ControlMode { VELOCITY, POSITION }

    public enum Controller { CUSTOM_CODE, HUB_BUILT_IN }

    public enum TestMode {
        HOLD, // stay on the selected target (X / Y to pick)
        STEP  // flip between TARGET_1 and TARGET_2 every STEP_TIME seconds
    }

    // Encoder ticks per output-shaft revolution, used to show RPM.
    public enum MotorType {
        GOBILDA_6000_RPM_BARE(28),
        GOBILDA_1620_RPM(103.8),
        GOBILDA_1150_RPM(145.1),
        GOBILDA_435_RPM(384.5),
        GOBILDA_312_RPM(537.7),
        GOBILDA_223_RPM(751.8),
        GOBILDA_117_RPM(1425.1),
        REV_HD_HEX_BARE(28),
        REV_HD_HEX_20_TO_1(560),
        REV_HD_HEX_40_TO_1(1120),
        REV_CORE_HEX(288),
        CUSTOM(0);

        final double ticksPerRev; // The number in brackets above.

        MotorType(double ticksPerRev) {
            this.ticksPerRev = ticksPerRev;
        }
    }

    // ======================= INTERNAL STATE (not in Panels) =======================
    private TelemetryManager panels;
    private DcMotorEx motor;                    // null if MOTOR_NAME wasn't found.
    private VoltageSensor voltageSensor;        // null if the robot has no voltage sensor.
    private String setupKey = "";               // what the motor was last set up for
    private PIDFCoefficients lastHubGains;      // last gains sent to the hub (avoid resending)

    private boolean useTarget2 = false;         // true = aim for TARGET_2.
    private double nudgeOffset = 0;             // Extra amount added by the D-pad.
    private double lastTarget = Double.NaN;     // Last loop's target (NaN = none yet).

    // PIDF memory between loops.
    private double integralSum = 0;             // Running total of error (I term).
    private double lastError = 0;               // Last loop's error (D term).
    private double filteredDerivative = 0;      // Smoothed D value.
    private double pOut, iOut, dOut, fOut;      // Each part's power, shown on the graph.
    private final ElapsedTime pidTimer = new ElapsedTime(); // Time between PIDF updates (dt).

    private double voltage = 12.0;                                // Last battery reading.
    private final ElapsedTime voltageTimer = new ElapsedTime();   // When we last read the battery.
    private final ElapsedTime stepTimer = new ElapsedTime();      // When STEP mode last flipped.
    private final ElapsedTime loopTimer = new ElapsedTime();      // Loop time, shown as "loop ms".

    // Step stats: measure how well the motor reached the latest target.
    private final ElapsedTime statTimer = new ElapsedTime();
    private double statStart, statTarget;
    private double riseTime = -1, settleTime = -1, maxOvershoot = 0;
    private double lastOutsideTime = 0;
    private double steadyErrorSum = 0;
    private int steadyErrorCount = 0;

    // F finder: average steady-state speed (peak alone is noisy and makes F too small).
    private boolean finderRunning = false;      // true while the motor is at full power measuring.
    private final ElapsedTime finderTimer = new ElapsedTime();
    private double finderVelocitySum = 0;       // ticks/s sum after spin-up
    private int finderVelocityCount = 0;
    private double finderPeakVelocity = 0;      // ticks/s peak (telemetry only)
    private String finderResult = "";

    // Gamepad edge detection (so holding a button only counts once)
    private boolean prevDpadUp, prevDpadDown, prevRightBumper, prevBack;

    // Runs once when INIT is pressed.
    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        // Read all hub sensors in one go each loop -> much faster loop, better tuning.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        // Always start with the motor off, even if Panels still has these on from last time.
        ENABLED = false;
        RUN_F_FINDER = false;
        setUpMotorIfChanged();
    }

    // Runs over and over after INIT, before PLAY.
    @Override
    public void init_loop() {
        setUpMotorIfChanged(); // lets you fix MOTOR_NAME in Panels while in INIT
        double measured = motor == null ? 0 : readMeasured();
        sendTelemetry(currentTarget(), measured, 0);
    }

    // Runs once when PLAY is pressed: start everything fresh.
    @Override
    public void start() {
        resetController();
        resetStats(0, currentTarget());
        stepTimer.reset();
        loopTimer.reset();
    }

    // Runs over and over while playing.
    @Override
    public void loop() {
        setUpMotorIfChanged(); // follow changes made in Panels (name, mode, controller ...)
        readGamepad();
        updateVoltage();

        if (motor == null) {
            sendTelemetry(0, 0, 0);
            return;
        }

        // ---- F finder takes over the motor while it runs ----
        if (RUN_F_FINDER && !finderRunning) startFinder();
        if (finderRunning) {
            runFinder();
            return;
        }

        // ---- STEP test: flip the target by itself ----
        if (TEST_MODE == TestMode.STEP && stepTimer.seconds() >= STEP_TIME) {
            useTarget2 = !useTarget2;
            stepTimer.reset();
        }

        double target = currentTarget();
        double measured = readMeasured();

        // New target -> start measuring a new step.
        if (target != lastTarget) {
            resetStats(measured, target);
            lastTarget = target;
        }

        if (!ENABLED) {
            stopMotor();
            resetController();
            sendTelemetry(target, measured, 0);
            return;
        }

        // Pick which controller drives the motor.
        double power;
        if (CONTROLLER == Controller.CUSTOM_CODE) {
            power = calculatePIDF(target, measured);
            motor.setPower(power);
        } else {
            power = runHubPIDF(target);
        }

        updateStats(measured);
        sendTelemetry(target, measured, power);
    }

    // Runs once when STOP is pressed.
    @Override
    public void stop() {
        stopMotor();
        ENABLED = false;
        RUN_F_FINDER = false;
    }

    // ============================================================================
    //  CONTROLLERS
    // ============================================================================

    // Our own PIDF. Returns motor power (-MAX_POWER..MAX_POWER).
    private double calculatePIDF(double target, double measured) {
        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02; // first loop / big pause: use a normal value

        double error = target - measured;

        // P: push harder the further off we are.
        pOut = P * error;

        // I: add up small errors over time. Clamped so it can't wind up huge.
        integralSum += error * dt;
        if (I != 0) {
            double limit = MAX_INTEGRAL_POWER / Math.abs(I);
            integralSum = Range.clip(integralSum, -limit, limit);
        } else {
            integralSum = 0;
        }
        iOut = I * integralSum;

        // D: slow down when closing in fast. Low-pass filtered because encoder data is noisy.
        double rawDerivative = (error - lastError) / dt;
        double filter = Range.clip(D_FILTER, 0, 0.99);
        filteredDerivative = filter * filteredDerivative + (1 - filter) * rawDerivative;
        dOut = D * filteredDerivative;
        lastError = error;

        // F (+ kS): feedforward = power we know we need before looking at the error.
        if (CONTROL_MODE == ControlMode.VELOCITY) {
            // Power ~ F * RPM. kS beats friction to get moving. Same units as Shooter kF.
            fOut = F * target + (target != 0 ? kS * Math.signum(target) : 0);
        } else {
            // F = constant hold power (gravity on a lift). kS = push to beat friction.
            fOut = F + (Math.abs(error) > TOLERANCE ? kS * Math.signum(error) : 0);
        }

        double output = pOut + iOut + dOut + fOut;

        // Same command gives less speed on a low battery. Scale up to match NOMINAL_VOLTAGE.
        if (VOLTAGE_COMPENSATION && voltage > 6) output *= NOMINAL_VOLTAGE / voltage;

        return Range.clip(output, -MAX_POWER, MAX_POWER);
    }

    // The hub's built-in PIDF. Returns the power the motor is actually using (for the graph).
    private double runHubPIDF(double target) {
        PIDFCoefficients gains = new PIDFCoefficients(P, I, D, F);
        if (!sameGains(gains, lastHubGains)) { // only send when changed, it's a slow command
            if (CONTROL_MODE == ControlMode.VELOCITY) {
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, gains);
            } else {
                // The hub's position loop only uses P (it feeds a velocity loop underneath).
                motor.setPositionPIDFCoefficients(P);
            }
            lastHubGains = gains;
        }

        if (CONTROL_MODE == ControlMode.VELOCITY) {
            // Hub velocity API is ticks/sec; Panels targets are RPM.
            motor.setVelocity(rpmToTicksPerSec(target));
        } else {
            motor.setTargetPosition((int) Math.round(target)); // already in RUN_TO_POSITION
            motor.setPower(MAX_POWER);
        }
        pOut = iOut = dOut = fOut = 0;
        return motor.getPower();
    }

    // Clears the PIDF memory so an old run doesn't affect the next one.
    private void resetController() {
        integralSum = 0;
        lastError = 0;
        filteredDerivative = 0;
        pOut = iOut = dOut = fOut = 0;
        pidTimer.reset();
    }

    // ============================================================================
    //  F FINDER (velocity only): full power, then average steady-state speed for F.
    // ============================================================================

    private void startFinder() {
        if (CONTROL_MODE != ControlMode.VELOCITY) {
            finderResult = "F finder only works in VELOCITY mode. For POSITION set F by hand.";
            RUN_F_FINDER = false;
            return;
        }
        finderRunning = true;
        finderVelocitySum = 0;
        finderVelocityCount = 0;
        finderPeakVelocity = 0;
        finderTimer.reset();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // raw power, no PID
    }

    private void runFinder() {
        motor.setPower(1.0);
        double velocityTicks = Math.abs(motor.getVelocity());
        double velocityRpm = toRpm(velocityTicks);
        // Ignore the first half second while it spins up; average the rest (not peak).
        if (finderTimer.seconds() > 0.5) {
            finderVelocitySum += velocityTicks;
            finderVelocityCount++;
            finderPeakVelocity = Math.max(finderPeakVelocity, velocityTicks);
        }

        double avgRpmSoFar = finderVelocityCount > 0 ? toRpm(finderVelocitySum / finderVelocityCount) : 0;
        panels.addLine("F FINDER RUNNING - motor at FULL POWER. Keep hands clear!");
        panels.addLine(String.format("Time %.1f / %.1f s", finderTimer.seconds(), F_FINDER_SECONDS));
        panels.addData("measured", velocityRpm);
        panels.addData("avg RPM", avgRpmSoFar);
        panels.addData("peak RPM", toRpm(finderPeakVelocity));
        panels.update(telemetry);

        if (finderTimer.seconds() < F_FINDER_SECONDS) return;

        // Done.
        motor.setPower(0);
        finderRunning = false;
        RUN_F_FINDER = false;
        setupKey = ""; // put the motor back in its normal mode next loop

        if (finderVelocityCount < 1 || finderVelocitySum < 1) {
            finderResult = "F finder saw no movement. Is the encoder plugged in?";
            return;
        }

        // Steady-state average ticks/s at full power. CUSTOM_CODE F = power per RPM (Shooter kF).
        double avgTicks = finderVelocitySum / finderVelocityCount;
        double avgRpm = toRpm(avgTicks);
        double peakRpm = toRpm(finderPeakVelocity);
        // Raw F matches Shooter (no voltage scale). Comp F only if VOLTAGE_COMPENSATION is on.
        double rawCustomF = avgRpm > 0 ? 1.0 / avgRpm : 0;
        double compCustomF = avgRpm > 0 ? (voltage / NOMINAL_VOLTAGE) / avgRpm : 0;
        double customF = VOLTAGE_COMPENSATION ? compCustomF : rawCustomF;
        double hubF = 32767.0 / avgTicks;
        double suggestedTarget = Math.round(avgRpm * 0.7);

        finderResult = String.format(
                "Avg %.0f RPM (peak %.0f) @ %.1fV. F raw=%.6f | F volt-comp=%.6f | applied=%s. "
                        + "HUB F=%.2f. Suggested TARGET_1~%.0f (not auto-changed).",
                avgRpm, peakRpm, voltage, rawCustomF, compCustomF,
                VOLTAGE_COMPENSATION ? "volt-comp" : "raw",
                hubF, suggestedTarget);

        if (F_FINDER_AUTO_APPLY) {
            F = CONTROLLER == Controller.CUSTOM_CODE ? customF : hubF;
        }
    }

    // ============================================================================
    //  STEP STATS: how good is the response to the latest target change?
    // ============================================================================

    // Starts measuring a new step from "from" to "to".
    private void resetStats(double from, double to) {
        statStart = from;
        statTarget = to;
        riseTime = -1;
        settleTime = -1;
        maxOvershoot = 0;
        lastOutsideTime = 0;
        steadyErrorSum = 0;
        steadyErrorCount = 0;
        statTimer.reset();
    }

    private void updateStats(double measured) {
        double t = statTimer.seconds();
        double stepSize = statTarget - statStart;
        double direction = Math.signum(stepSize);
        double error = statTarget - measured;

        // Rise time: first time we get 90% of the way there.
        if (riseTime < 0 && stepSize != 0
                && (measured - statStart) * direction >= 0.9 * Math.abs(stepSize)) {
            riseTime = t;
        }

        // Overshoot: how far we went PAST the target, as % of the step.
        if (stepSize != 0) {
            double past = (measured - statTarget) * direction;
            maxOvershoot = Math.max(maxOvershoot, past / Math.abs(stepSize) * 100.0);
        }

        // Settling: within TOLERANCE and stayed there for SETTLE_TIME.
        if (Math.abs(error) > TOLERANCE) {
            lastOutsideTime = t;
            if (settleTime >= 0) settleTime = -1; // knocked out again
        } else if (settleTime < 0 && t - lastOutsideTime >= SETTLE_TIME) {
            settleTime = lastOutsideTime;
        }

        // Steady-state error: average error once settled.
        if (settleTime >= 0) {
            steadyErrorSum += error;
            steadyErrorCount++;
        }
    }

    // ============================================================================
    //  HELPERS
    // ============================================================================

    // (Re)finds and sets up the motor when MOTOR_NAME, the mode, or the controller changes.
    private void setUpMotorIfChanged() {
        // MOTOR_TYPE is in the key so changing it refreshes RPM conversion without a restart.
        String key = MOTOR_NAME + "|" + CONTROL_MODE + "|" + CONTROLLER + "|" + MOTOR_TYPE
                + "|" + CUSTOM_TICKS_PER_REV;
        if (key.equals(setupKey)) {
            if (motor != null) applyDirection();
            return;
        }

        if (motor != null) motor.setPower(0);
        setupKey = key;
        lastHubGains = null;
        lastTarget = Double.NaN;
        resetController();

        motor = hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAME); // null if the name is wrong
        if (motor == null) return;

        applyDirection();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (CONTROLLER == Controller.CUSTOM_CODE) {
            // We do the PIDF ourselves, so turn the hub's off (the encoder still counts).
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (CONTROL_MODE == ControlMode.VELOCITY) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // Flywheels should coast; arms/lifts should hold.
        motor.setZeroPowerBehavior(CONTROL_MODE == ControlMode.POSITION
                ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // Sets the motor direction from REVERSED (only if it actually changed).
    private void applyDirection() {
        DcMotorSimple.Direction want = REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        if (motor.getDirection() != want) motor.setDirection(want);
    }

    // Turns the motor off, for either controller.
    private void stopMotor() {
        if (motor == null) return;
        if (CONTROLLER == Controller.HUB_BUILT_IN && CONTROL_MODE == ControlMode.VELOCITY) {
            motor.setVelocity(0);
        }
        motor.setPower(0);
    }

    // Checks the gamepad buttons (see GAMEPAD 1 CONTROLS at the top).
    private void readGamepad() {
        if (gamepad1.a) ENABLED = true;
        if (gamepad1.b) ENABLED = false;
        if (gamepad1.x) useTarget2 = false;
        if (gamepad1.y) useTarget2 = true;

        if (gamepad1.dpad_up && !prevDpadUp) nudgeOffset += NUDGE;
        if (gamepad1.dpad_down && !prevDpadDown) nudgeOffset -= NUDGE;
        if (gamepad1.right_bumper && !prevRightBumper) RUN_F_FINDER = true;
        if (gamepad1.back && !prevBack) resetStats(motor == null ? 0 : readMeasured(), currentTarget());

        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevRightBumper = gamepad1.right_bumper;
        prevBack = gamepad1.back;
    }

    // The target we're aiming for right now (TARGET_1 or TARGET_2, plus any D-pad nudge).
    private double currentTarget() {
        return (useTarget2 ? TARGET_2 : TARGET_1) + nudgeOffset;
    }

    // Where the motor actually is: RPM in VELOCITY mode, encoder ticks in POSITION mode.
    private double readMeasured() {
        return CONTROL_MODE == ControlMode.VELOCITY
                ? toRpm(motor.getVelocity())
                : motor.getCurrentPosition();
    }

    private void updateVoltage() {
        // Reading voltage is slow, so only do it 4 times a second.
        if (voltageSensor != null && voltageTimer.seconds() > 0.25) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
    }

    // Encoder ticks per turn for the chosen MOTOR_TYPE.
    private double ticksPerRev() {
        return MOTOR_TYPE == MotorType.CUSTOM ? CUSTOM_TICKS_PER_REV : MOTOR_TYPE.ticksPerRev;
    }

    // Converts ticks/sec to RPM: divide by ticks per turn, times 60 seconds.
    private double toRpm(double ticksPerSecond) {
        double tpr = ticksPerRev();
        return tpr > 0 ? ticksPerSecond / tpr * 60.0 : 0;
    }

    // Converts RPM to ticks/sec for the hub velocity API.
    private double rpmToTicksPerSec(double rpm) {
        double tpr = ticksPerRev();
        return tpr > 0 ? rpm * tpr / 60.0 : 0;
    }

    // true if two sets of gains are exactly the same.
    private static boolean sameGains(PIDFCoefficients a, PIDFCoefficients b) {
        return b != null && a.p == b.p && a.i == b.i && a.d == b.d && a.f == b.f;
    }

    // Formats a time like "0.42s", or "--" if it hasn't happened yet (-1).
    private static String time(double seconds) {
        return seconds < 0 ? "--" : String.format("%.2fs", seconds);
    }

    // ============================================================================
    //  TELEMETRY: shows in Panels (Telemetry + Graph tabs) AND on the Driver Station
    // ============================================================================

    private void sendTelemetry(double target, double measured, double power) {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        String units = CONTROL_MODE == ControlMode.VELOCITY ? "RPM" : "ticks";

        // ---- Status ----
        if (motor == null) {
            panels.addLine("!! Motor: NOT FOUND \"" + MOTOR_NAME + "\"");
            panels.addLine("   Fix MOTOR_NAME in Panels > Configurables (must match the robot config).");
        } else {
            panels.addLine("Motor: OK \"" + MOTOR_NAME + "\" | " + CONTROL_MODE + " (" + units + ") | "
                    + CONTROLLER + " | " + TEST_MODE);
            panels.addLine("Controller: " + (ENABLED ? "ON (B = off)" : "OFF (press A to turn on)")
                    + " | Target " + (useTarget2 ? "2" : "1") + (nudgeOffset != 0 ? " + nudge " + nudgeOffset : ""));
        }

        // ---- Graphable numbers (add these in Panels > Graph) ----
        // VELOCITY: target/measured/error are RPM. POSITION: encoder ticks.
        panels.addData("target", target);
        panels.addData("measured", measured);
        panels.addData("error", target - measured);
        panels.addData("power", power);
        if (CONTROLLER == Controller.CUSTOM_CODE) {
            panels.addData("pTerm", pOut);
            panels.addData("iTerm", iOut);
            panels.addData("dTerm", dOut);
            panels.addData("fTerm", fOut);
        }
        if (CONTROL_MODE == ControlMode.VELOCITY) {
            panels.addData("target ticks/s", rpmToTicksPerSec(target));
            panels.addData("measured ticks/s", rpmToTicksPerSec(measured));
        } else {
            panels.addData("measured revs", ticksPerRev() > 0 ? measured / ticksPerRev() : 0);
        }
        panels.addData("voltage", voltage);
        panels.addData("loop ms", loopMs);

        // ---- Step results ----
        panels.addLine(String.format("STEP %.0f -> %.0f %s | rise %s | overshoot %.1f%% | settle %s | steady err %s",
                statStart, statTarget, units, time(riseTime), maxOvershoot,
                settleTime < 0 ? "not yet" : time(settleTime),
                steadyErrorCount == 0 ? "--" : String.format("%.1f", steadyErrorSum / steadyErrorCount)));

        // ---- Warnings / hints based on what we see ----
        if (ENABLED && motor != null) {
            if (Math.abs(power) >= MAX_POWER - 0.001) {
                panels.addLine("Hint: power is maxed out. Target may be too high, or gains too big.");
            }
            if (maxOvershoot > 15) {
                panels.addLine("Hint: big overshoot. Lower P or add D.");
            }
            if (settleTime < 0 && statTimer.seconds() > 3 && riseTime >= 0) {
                panels.addLine("Hint: not settling (oscillating?). Lower P / I, or add D.");
            }
            if (riseTime < 0 && statTimer.seconds() > 3) {
                panels.addLine("Hint: never reached 90% of target. Raise F / P (or check REVERSED).");
            }
            if (CONTROL_MODE == ControlMode.VELOCITY && Math.signum(measured) == -Math.signum(target)
                    && Math.abs(measured) > TOLERANCE) {
                panels.addLine("Hint: spinning the WRONG way. Flip REVERSED.");
            }
        }
        if (!finderResult.isEmpty()) panels.addLine("F finder: " + finderResult);

        // ---- Transfer warnings (Shooter / NextFTC has no voltage scale and ignores kS) ----
        if (CONTROLLER == Controller.CUSTOM_CODE && CONTROL_MODE == ControlMode.VELOCITY) {
            if (VOLTAGE_COMPENSATION) {
                panels.addLine("WARN: VOLTAGE_COMPENSATION is ON — Shooter.java does NOT. Gains will not match teleop.");
            }
            if (kS != 0) {
                panels.addLine("WARN: kS != 0 — Shooter.java does not set kS. Leave kS=0 or add setKS in Shooter.");
            }
            if (D != 0 && D_FILTER > 0) {
                panels.addLine("WARN: D is filtered here (D_FILTER) but not in NextFTC — expect a stronger D in Shooter.");
            }
        }

        // ---- Copy-paste line ----
        if (CONTROLLER == Controller.CUSTOM_CODE) {
            if (CONTROL_MODE == ControlMode.VELOCITY) {
                // Shooter uses kV for F and currently has no kS field wired.
                panels.addLine(String.format(
                        "COPY THIS (Shooter): kP=%s, kI=%s, kD=%s, kF=%s",
                        P, I, D, F));
                panels.addLine(String.format(
                        "COPY THIS (full CUSTOM_CODE): kP=%s, kI=%s, kD=%s, kF=%s, kS=%s",
                        P, I, D, F, kS));
            } else {
                panels.addLine(String.format(
                        "COPY THIS (ticks): P=%s, I=%s, D=%s, F=%s, kS=%s",
                        P, I, D, F, kS));
            }
        } else if (CONTROL_MODE == ControlMode.VELOCITY) {
            panels.addLine(String.format("COPY THIS: motor.setVelocityPIDFCoefficients(%s, %s, %s, %s);",
                    P, I, D, F));
        } else {
            panels.addLine(String.format(
                    "COPY THIS (hub position uses P only): motor.setPositionPIDFCoefficients(%s);", P));
        }

        // ---- Help ----
        if (SHOW_HELP) {
            panels.addLine("--- HOW TO USE (SHOW_HELP = false to hide) ---");
            panels.addLine("Goal: 'measured' jumps to 'target' fast and stays flat. F=guess P=push I=leftover D=brakes");
            panels.addLine("gp1: A on | B off | X target1 | Y target2 | dpad +/- nudge | RB F finder | back reset stats");
            if (CONTROL_MODE == ControlMode.VELOCITY) {
                panels.addLine("VELOCITY = RPM. Keep VOLTAGE_COMPENSATION off + kS=0 to match Shooter.java.");
                panels.addLine("1) P=I=D=0, press RB to find F  2) A to turn on  3) raise P till fast, back off if it buzzes");
                panels.addLine("4) tiny I only if it stays under target  5) D usually 0 for flywheels");
            } else {
                panels.addLine("POSITION units = encoder ticks.");
                panels.addLine("1) F = power that holds it still, kS = power that just moves it  2) A to turn on");
                panels.addLine("3) raise P till it gets there fast  4) add D to stop overshoot  5) tiny I if it stops short");
            }
            panels.addLine("Panels > Graph: plot target + measured. TEST_MODE=STEP flips targets every STEP_TIME s.");
            panels.addLine("Values reset on restart - copy the COPY THIS line into your code!");
        }

        panels.update(telemetry);
    }
}
