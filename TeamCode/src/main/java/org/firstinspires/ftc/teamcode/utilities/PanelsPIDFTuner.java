package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * ============================================================================
 *  PANELS PIDF TUNER  -  tune the PIDF of ANY motor, live, from Panels
 * ============================================================================
 *
 * WHAT IT DOES
 *   You type a motor's config name into Panels. This OpMode drives that motor
 *   with a PIDF controller and graphs how well it follows the target, so you can
 *   change P, I, D, F live and see the result right away. It also:
 *     - Finds F for you (the "F finder" spins the motor at full power and measures top speed).
 *     - Runs an automatic STEP test (flips between two targets) so you don't need hands.
 *     - Measures rise time, overshoot, settling time and steady-state error for every step.
 *     - Can use YOUR code's PIDF (CUSTOM_CODE) or the hub's built-in PIDF (HUB_BUILT_IN).
 *     - Optional battery voltage compensation, kS (static friction), integral limit, D filter.
 *
 * SETUP (once)
 *   1. Connect the laptop to the robot's Wi-Fi.
 *   2. Open Panels in a browser: http://192.168.43.1:8001
 *   3. On the Driver Station pick TeleOp "Panels PIDF Tuner" (group "Tuning") and press INIT.
 *   4. In Panels > Configurables > PanelsPIDFTuner set:
 *        MOTOR_NAME     = the motor's name in the robot configuration (exact spelling!)
 *        MOTOR_TYPE     = your motor (or CUSTOM and fill CUSTOM_TICKS_PER_REV)
 *        CONTROL_MODE   = VELOCITY (flywheels/shooters/intakes) or POSITION (arms/lifts/turrets)
 *        CONTROLLER     = CUSTOM_CODE (recommended) or HUB_BUILT_IN
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
 * TUNING RECIPE
 *   VELOCITY (flywheel):
 *     1. Set P = I = D = 0. Run the F finder (right bumper). It fills in F for you.
 *     2. Turn on with A. "measured" should land close to "target" with F alone.
 *     3. Raise P until it gets to target fast. If it oscillates/buzzes, back P off ~30%.
 *     4. If it still sits a little under target, add a tiny I (start ~ P / 10).
 *     5. D is rarely needed for flywheels. Leave it 0 unless it overshoots a lot.
 *   POSITION (arm / lift):
 *     1. Set everything to 0. Set F to the power that just barely holds the arm/lift still
 *        against gravity (lifts). Set kS to the power that just barely gets it moving.
 *     2. Raise P until it gets to target quickly with a little overshoot.
 *     3. Add D to remove the overshoot / wobble.
 *     4. Add a small I only if it stops short of target.
 *   Use TEST_MODE = STEP to flip between TARGET_1 and TARGET_2 automatically and
 *   watch the "step" numbers: lower rise / settle time and ~0% overshoot is the goal.
 *
 * WHEN YOU'RE DONE
 *   Panels changes are LOST when the robot restarts! Copy the "COPY THIS" line from
 *   telemetry into your real code.
 *
 * GAIN UNITS (why the numbers look tiny)
 *   CUSTOM_CODE: error is in ticks (POSITION) or ticks/sec (VELOCITY), output is motor
 *   power (-1..1). So a velocity P of 0.002 means "0.002 power per tick/sec off".
 *   HUB_BUILT_IN: the hub uses its own (much bigger) units, roughly F = 32767 / max ticks/sec.
 *   Gains from one controller DO NOT work in the other.
 * ============================================================================
 */
@Configurable // Makes every public static field below show up (and be editable) in Panels.
@TeleOp(name = "Panels PIDF Tuner", group = "Tuning")
public class PanelsPIDFTuner extends OpMode {

    // ======================= INSTRUCTIONS (shown in Panels) =======================
    // These are only here so the steps show up inside Panels > Configurables.
    // Changing them does nothing.
    public static String HOW_TO_1 = "Type the motor's config name in MOTOR_NAME, pick MOTOR_TYPE + CONTROL_MODE";
    public static String HOW_TO_2 = "Graph tab: add 'target' and 'measured'. Press START, then A on gamepad1 to turn on";
    public static String HOW_TO_3 = "VELOCITY: run F finder (RUN_F_FINDER or right bumper) -> raise P -> tiny I if short";
    public static String HOW_TO_4 = "POSITION: F = hold power, kS = friction, raise P, add D to stop overshoot";
    public static String HOW_TO_5 = "TEST_MODE = STEP flips targets by itself. Watch rise/overshoot/settle in telemetry";
    public static String HOW_TO_6 = "Copy the 'COPY THIS' line into your code. Panels values reset on restart!";

    // ======================= MOTOR =======================
    public static String MOTOR_NAME = "motor";
    public static boolean REVERSED = false;
    public static MotorType MOTOR_TYPE = MotorType.GOBILDA_6000_RPM_BARE;
    public static double CUSTOM_TICKS_PER_REV = 28; // only used when MOTOR_TYPE = CUSTOM

    // ======================= MODE =======================
    public static ControlMode CONTROL_MODE = ControlMode.VELOCITY;
    public static Controller CONTROLLER = Controller.CUSTOM_CODE;
    public static TestMode TEST_MODE = TestMode.HOLD;
    public static boolean ENABLED = false;     // same as A / B on the gamepad

    // ======================= GAINS =======================
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;
    public static double kS = 0.0;              // CUSTOM_CODE only: power to beat friction

    // ======================= TARGETS =======================
    // VELOCITY = ticks per second, POSITION = ticks.
    public static double TARGET_1 = 1500;
    public static double TARGET_2 = 0;
    public static double STEP_TIME = 2.0;       // seconds between flips in STEP mode
    public static double NUDGE = 100;           // dpad up/down changes the target by this much

    // ======================= SAFETY / EXTRAS (CUSTOM_CODE only) =======================
    public static double MAX_POWER = 1.0;
    public static double MAX_INTEGRAL_POWER = 0.25; // most power the I term may add
    public static double D_FILTER = 0.7;         // 0 = no filter, 0.9 = heavy smoothing of D
    public static boolean VOLTAGE_COMPENSATION = true;
    public static double NOMINAL_VOLTAGE = 12.0;

    // ======================= STATS =======================
    public static double TOLERANCE = 50;         // "close enough" (ticks or ticks/sec)
    public static double SETTLE_TIME = 0.3;      // must stay in tolerance this long to count as settled

    // ======================= F FINDER =======================
    public static boolean RUN_F_FINDER = false;  // set true in Panels (or right bumper) to run it
    public static double F_FINDER_SECONDS = 2.5;
    public static boolean F_FINDER_AUTO_APPLY = true; // put the found F straight into F

    // ======================= TELEMETRY =======================
    public static boolean SHOW_HELP = true;       // show the how-to lines in telemetry

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

        final double ticksPerRev;

        MotorType(double ticksPerRev) {
            this.ticksPerRev = ticksPerRev;
        }
    }

    // ======================= INTERNAL STATE (not in Panels) =======================
    private TelemetryManager panels;
    private DcMotorEx motor;
    private VoltageSensor voltageSensor;
    private String setupKey = "";               // what the motor was last set up for
    private PIDFCoefficients lastHubGains;      // last gains sent to the hub (avoid resending)

    private boolean useTarget2 = false;
    private double nudgeOffset = 0;
    private double lastTarget = Double.NaN;

    private double integralSum = 0;
    private double lastError = 0;
    private double filteredDerivative = 0;
    private double pOut, iOut, dOut, fOut;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private double voltage = 12.0;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Step stats
    private final ElapsedTime statTimer = new ElapsedTime();
    private double statStart, statTarget;
    private double riseTime = -1, settleTime = -1, maxOvershoot = 0;
    private double lastOutsideTime = 0;
    private double steadyErrorSum = 0;
    private int steadyErrorCount = 0;

    // F finder
    private boolean finderRunning = false;
    private final ElapsedTime finderTimer = new ElapsedTime();
    private double finderMaxVelocity = 0;
    private String finderResult = "";

    // Gamepad edge detection (so holding a button only counts once)
    private boolean prevDpadUp, prevDpadDown, prevRightBumper, prevBack;

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

        ENABLED = false;
        RUN_F_FINDER = false;
        setUpMotorIfChanged();
    }

    @Override
    public void init_loop() {
        setUpMotorIfChanged(); // lets you fix MOTOR_NAME in Panels while in INIT
        double measured = motor == null ? 0 : readMeasured();
        sendTelemetry(currentTarget(), measured, 0);
    }

    @Override
    public void start() {
        resetController();
        resetStats(0, currentTarget());
        stepTimer.reset();
        loopTimer.reset();
    }

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
            // Power needed is about proportional to speed. kS beats friction to get moving.
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
            motor.setVelocity(target);
        } else {
            motor.setTargetPosition((int) Math.round(target)); // already in RUN_TO_POSITION
            motor.setPower(MAX_POWER);
        }
        pOut = iOut = dOut = fOut = 0;
        return motor.getPower();
    }

    private void resetController() {
        integralSum = 0;
        lastError = 0;
        filteredDerivative = 0;
        pOut = iOut = dOut = fOut = 0;
        pidTimer.reset();
    }

    // ============================================================================
    //  F FINDER (velocity only): full power for a few seconds, measure top speed.
    // ============================================================================

    private void startFinder() {
        if (CONTROL_MODE != ControlMode.VELOCITY) {
            finderResult = "F finder only works in VELOCITY mode. For POSITION set F by hand.";
            RUN_F_FINDER = false;
            return;
        }
        finderRunning = true;
        finderMaxVelocity = 0;
        finderTimer.reset();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // raw power, no PID
    }

    private void runFinder() {
        motor.setPower(1.0);
        double velocity = Math.abs(motor.getVelocity());
        // Ignore the first half second while it spins up.
        if (finderTimer.seconds() > 0.5) finderMaxVelocity = Math.max(finderMaxVelocity, velocity);

        panels.addLine("F FINDER RUNNING - motor at FULL POWER. Keep hands clear!");
        panels.addLine(String.format("Time %.1f / %.1f s", finderTimer.seconds(), F_FINDER_SECONDS));
        panels.addData("measured", velocity);
        panels.addData("max velocity", finderMaxVelocity);
        panels.update(telemetry);

        if (finderTimer.seconds() < F_FINDER_SECONDS) return;

        // Done.
        motor.setPower(0);
        finderRunning = false;
        RUN_F_FINDER = false;
        setupKey = ""; // put the motor back in its normal mode next loop

        if (finderMaxVelocity < 1) {
            finderResult = "F finder saw no movement. Is the encoder plugged in?";
            return;
        }
        // Full power gave finderMaxVelocity. With voltage compensation the output gets
        // scaled by NOMINAL/voltage, so measure F relative to NOMINAL_VOLTAGE.
        double fullPowerAtNominal = VOLTAGE_COMPENSATION ? voltage / NOMINAL_VOLTAGE : 1.0;
        double customF = fullPowerAtNominal / finderMaxVelocity;
        double hubF = 32767.0 / finderMaxVelocity;

        finderResult = String.format(
                "Max speed %.0f ticks/s (%.0f RPM) @ %.1fV. CUSTOM_CODE F=%.6f | HUB_BUILT_IN F=%.2f, P~%.2f, I~%.3f",
                finderMaxVelocity, toRpm(finderMaxVelocity), voltage, customF, hubF, hubF * 0.1, hubF * 0.01);

        if (F_FINDER_AUTO_APPLY) {
            F = CONTROLLER == Controller.CUSTOM_CODE ? customF : hubF;
            TARGET_1 = Math.round(finderMaxVelocity * 0.7); // a sensible target to test with
        }
    }

    // ============================================================================
    //  STEP STATS: how good is the response to the latest target change?
    // ============================================================================

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
        String key = MOTOR_NAME + "|" + CONTROL_MODE + "|" + CONTROLLER;
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

    private void applyDirection() {
        DcMotorSimple.Direction want = REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        if (motor.getDirection() != want) motor.setDirection(want);
    }

    private void stopMotor() {
        if (motor == null) return;
        if (CONTROLLER == Controller.HUB_BUILT_IN && CONTROL_MODE == ControlMode.VELOCITY) {
            motor.setVelocity(0);
        }
        motor.setPower(0);
    }

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

    private double currentTarget() {
        return (useTarget2 ? TARGET_2 : TARGET_1) + nudgeOffset;
    }

    private double readMeasured() {
        return CONTROL_MODE == ControlMode.VELOCITY ? motor.getVelocity() : motor.getCurrentPosition();
    }

    private void updateVoltage() {
        // Reading voltage is slow, so only do it 4 times a second.
        if (voltageSensor != null && voltageTimer.seconds() > 0.25) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
    }

    private double ticksPerRev() {
        return MOTOR_TYPE == MotorType.CUSTOM ? CUSTOM_TICKS_PER_REV : MOTOR_TYPE.ticksPerRev;
    }

    private double toRpm(double ticksPerSecond) {
        double tpr = ticksPerRev();
        return tpr > 0 ? ticksPerSecond / tpr * 60.0 : 0;
    }

    private static boolean sameGains(PIDFCoefficients a, PIDFCoefficients b) {
        return b != null && a.p == b.p && a.i == b.i && a.d == b.d && a.f == b.f;
    }

    private static String time(double seconds) {
        return seconds < 0 ? "--" : String.format("%.2fs", seconds);
    }

    // ============================================================================
    //  TELEMETRY: shows in Panels (Telemetry + Graph tabs) AND on the Driver Station
    // ============================================================================

    private void sendTelemetry(double target, double measured, double power) {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        String units = CONTROL_MODE == ControlMode.VELOCITY ? "ticks/s" : "ticks";

        // ---- Status ----
        if (motor == null) {
            panels.addLine("!! Motor: NOT FOUND \"" + MOTOR_NAME + "\"");
            panels.addLine("   Fix MOTOR_NAME in Panels > Configurables (must match the robot config).");
        } else {
            panels.addLine("Motor: OK \"" + MOTOR_NAME + "\" | " + CONTROL_MODE + " | " + CONTROLLER
                    + " | " + TEST_MODE);
            panels.addLine("Controller: " + (ENABLED ? "ON (B = off)" : "OFF (press A to turn on)")
                    + " | Target " + (useTarget2 ? "2" : "1") + (nudgeOffset != 0 ? " + nudge " + nudgeOffset : ""));
        }

        // ---- Graphable numbers (add these in Panels > Graph) ----
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
            panels.addData("target RPM", toRpm(target));
            panels.addData("measured RPM", toRpm(measured));
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

        // ---- Copy-paste line ----
        if (CONTROLLER == Controller.CUSTOM_CODE) {
            panels.addLine(String.format("COPY THIS: P=%s, I=%s, D=%s, F=%s, kS=%s",
                    P, I, D, F, kS));
        } else {
            panels.addLine(String.format("COPY THIS: motor.setVelocityPIDFCoefficients(%s, %s, %s, %s);",
                    P, I, D, F));
        }

        // ---- Help ----
        if (SHOW_HELP) {
            panels.addLine("--- HOW TO USE (SHOW_HELP = false to hide) ---");
            panels.addLine("gp1: A on | B off | X target1 | Y target2 | dpad +/- nudge | RB F finder | back reset stats");
            if (CONTROL_MODE == ControlMode.VELOCITY) {
                panels.addLine("1) P=I=D=0, press RB to find F  2) A to turn on  3) raise P till fast, back off if it buzzes");
                panels.addLine("4) tiny I only if it stays under target  5) D usually 0 for flywheels");
            } else {
                panels.addLine("1) F = power that holds it still, kS = power that just moves it  2) A to turn on");
                panels.addLine("3) raise P till it gets there fast  4) add D to stop overshoot  5) tiny I if it stops short");
            }
            panels.addLine("Panels > Graph: plot target + measured. TEST_MODE=STEP flips targets every STEP_TIME s.");
            panels.addLine("Values reset on restart - copy the COPY THIS line into your code!");
        }

        panels.update(telemetry);
    }
}
