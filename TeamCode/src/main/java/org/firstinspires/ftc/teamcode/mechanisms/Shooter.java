package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;

/*
 * SHOOTER MECHANISM
 *
 * HARDWARE (names must match the robot configuration on the Driver Station):
 *   ShooterMotor     = motor that spins the shooter wheel
 *   ShooterGateServo = servo that blocks/unblocks the path into the shooter
 *
 * HOW IT WORKS:
 *   - The motor spins up when PLAY is pressed (not during INIT).
 *   - The gate starts CLOSED and flips between CLOSED and OPEN each time B is pressed.
 *
 * A "Mechanism" is NextFTC's name for one part of the robot. It holds the hardware
 * and gives back Commands (small actions) that the TeleOp can run or bind to buttons.
 */
public class Shooter implements Mechanism {

    // ---- Settings ----
    public static double SHOOTER_POWER = 1.0;
    public static double GATE_CLOSED = 0.5;
    public static double GATE_OPEN = 1.0;

    // ---- Hardware ----
    // NextMotor/NextServo don't touch the hardwareMap until they're first used,
    // so it's safe to create them right here.
    private final NextMotor shooterMotor = new NextMotor("ShooterMotor");
    // -1 turns off NextFTC's "skip if same position" caching, so the gate is always
    // re-sent its position at the start of every OpMode (the SDK relaxes servos on STOP).
    private final NextServo gateServo = new NextServo("ShooterGateServo", -1.0);

    private boolean gateOpen = false;

    // ---- Commands ----

    /** Spins the shooter wheel at full power. */
    public Command spinUp() {
        return instant(() -> shooterMotor.setThrottle(SHOOTER_POWER));
    }

    /** Stops the shooter wheel. */
    public Command stop() {
        return instant(() -> shooterMotor.setThrottle(0.0));
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
        return instant(() -> setGate(!gateOpen));
    }

    /**
     * Stops the motor right now (not as a command). Use this in an OpMode's end(),
     * because commands don't get a chance to run after STOP is pressed.
     */
    public void stopNow() {
        shooterMotor.setThrottle(0.0);
        shooterMotor.update();
    }

    private void setGate(boolean open) {
        gateOpen = open;
        gateServo.setPosition(open ? GATE_OPEN : GATE_CLOSED);
    }

    // Runs every loop automatically, so we use it to show what the shooter is doing.
    @Override
    public void periodic() {
        Telemetry.log("Shooter power", shooterMotor.getThrottle());
        Telemetry.log("Shooter gate", gateOpen ? "OPEN" : "CLOSED");
    }
}
