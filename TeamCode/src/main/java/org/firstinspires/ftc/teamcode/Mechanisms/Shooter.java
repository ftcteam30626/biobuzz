package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.ivy.Command;              // A small action the robot can run (e.g. "spin up").

import dev.nextftc.hardware.actuators.NextMotor;  // NextFTC's motor (has built-in PIDF speed control).
import dev.nextftc.hardware.actuators.NextServo;  // NextFTC's servo.
import dev.nextftc.robot.Mechanism;               // Marks this class as one part of the robot.
import dev.nextftc.robot.Telemetry;               // Sends text/numbers to the Driver Station screen.
import dev.nextftc.units.Units;                   // Lets us say "RPM" or "rotations" instead of raw numbers.

/*
 * SHOOTER MECHANISM
 *
 * HARDWARE (names must match the robot configuration on the Driver Station):
 *   ShooterMotor     = goBILDA bare motor (6000 RPM) that spins the shooter wheel
 *   ShooterGateServo = servo that blocks/unblocks the path into the shooter
 *
 * HOW IT WORKS:
 *   - The motor spins up to TARGET_RPM when PLAY is pressed (not during INIT).
 *   - A PIDF controller keeps the wheel at TARGET_RPM, even when a ball slows it down:
 *       F = the power we already know it needs for that speed (the "guess")
 *       P = extra push based on how far off the speed is right now
 *       I = extra push if it stays a little off for a while
 *       D = calms it down if the speed is changing fast
 *   - The gate starts CLOSED and flips between CLOSED and OPEN each time B is pressed.
 *
 * A "Mechanism" is NextFTC's name for one part of the robot. It holds the hardware
 * and gives back Commands (small actions) that the TeleOp can run or bind to buttons.
 */
public class Shooter implements Mechanism {

    // ---- Settings ----
    // "public static" = one shared value you can change from anywhere (or from Panels later).
    public static double TARGET_RPM = 2050;  // Speed the wheel tries to hold.
    public static double GATE_CLOSED = 0.5;  // Servo position that blocks the balls.
    public static double GATE_OPEN = 1.0;    // Servo position that lets balls through.

    // ---- PIDF gains (PLACEHOLDERS - tune these!) ----
    // Units: motor power (0 to 1) per RPM. That's why they're so small.
    public static double kP = 0.005;     // 200 RPM too slow -> adds 0.1 power.
    public static double kI = 0.0;        // Usually not needed on a flywheel.
    public static double kD = 0.0;        // Usually not needed on a flywheel.
    public static double kF = 0.00025; // Full power ~= 6000 RPM, so power = RPM / 6000.

    private static final double TICKS_PER_REV = 28.0; // Bare motor encoder ticks per turn.

    // ---- Hardware ----
    // NextMotor/NextServo don't touch the hardwareMap until they're first used,
    // so it's safe to create them right here.
    // The second value tells NextFTC that 1 encoder tick = 1/28 of a turn, so it can work in RPM.
    private final NextMotor shooterMotor =
            new NextMotor("ShooterMotor", Units.getRotations(1.0 / TICKS_PER_REV));
    // -1 turns off NextFTC's "skip if same position" caching, so the gate is always
    // re-sent its position at the start of every OpMode (the SDK relaxes servos on STOP).
    private final NextServo gateServo = new NextServo("ShooterGateServo", -1.0);

    // ---- State (what the shooter is doing right now) ----
    private boolean spinning = false;  // true while the wheel is supposed to be at TARGET_RPM.
    private boolean gateOpen = false;  // true while the gate is open.

    // ---- Commands ----
    // instant(...) makes a Command that runs the code inside it once, right away.
    // "() -> { ... }" is a lambda: a small chunk of code saved to run later.

    /** Spins the shooter wheel up to TARGET_RPM and holds it there with PIDF. */
    public Command spinUp() {
        return instant(() -> {
            spinning = true;
            // Switch the motor to speed control. NextFTC now runs the PIDF every loop.
            shooterMotor.setVelocitySetpoint(Units.getRpm(TARGET_RPM));
        });
    }

    /** Stops the shooter wheel (it coasts to a stop). */
    public Command stop() {
        return instant(this::stopMotor); // "this::stopMotor" = "run stopMotor()".
    }

    /** Moves the gate to CLOSED (0.5). */
    public Command closeGate() {
        return instant(() -> setGate(false));
    }

    /** Moves the gate to OPEN (1.0). */
    public Command openGate() {
        return instant(() -> setGate(true));
    }

    /** Flips the gate: CLOSED -> OPEN, or OPEN -> CLOSED. */
    public Command toggleGate() {
        return instant(() -> setGate(!gateOpen)); // "!" flips true <-> false.
    }

    /**
     * Stops the motor right now (not as a command). Use this in an OpMode's end(),
     * because commands don't get a chance to run after STOP is pressed.
     */
    public void stopNow() {
        stopMotor();
        shooterMotor.update(); // Send the 0 power to the motor immediately.
    }

    /** How fast the wheel is spinning right now, in RPM. */
    public double getRpm() {
        return shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    // ---- Helpers (only used inside this class) ----

    private void stopMotor() {
        spinning = false;
        shooterMotor.setThrottle(0.0); // Plain 0 power, so the PIDF is off and the wheel coasts.
    }

    private void setGate(boolean open) {
        gateOpen = open;
        // "a ? b : c" means "if a, use b, otherwise use c".
        gateServo.setPosition(open ? GATE_OPEN : GATE_CLOSED);
    }

    // Runs every loop automatically.
    @Override
    public void periodic() {
        // Copy the gains in every loop, so changing them while running takes effect right away.
        shooterMotor.getVelocityConstants().setKP(kP);
        shooterMotor.getVelocityConstants().setKI(kI);
        shooterMotor.getVelocityConstants().setKD(kD);
        shooterMotor.getVelocityConstants().setKV(kF); // NextFTC calls F "kV".

        // Show what the shooter is doing on the Driver Station.
        Telemetry.log("Shooter target RPM", spinning ? TARGET_RPM : 0);
        Telemetry.log("Shooter RPM", getRpm());
        Telemetry.log("Shooter power", shooterMotor.getThrottle());
        Telemetry.log("Shooter gate", gateOpen ? "OPEN" : "CLOSED");
    }
}
