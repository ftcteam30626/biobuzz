package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.ivy.Command;              // A small action the robot can run (e.g. "turn on").

import dev.nextftc.hardware.actuators.NextMotor;  // NextFTC's motor.
import dev.nextftc.robot.Mechanism;               // Marks this class as one part of the robot.
import dev.nextftc.robot.Telemetry;               // Sends text/numbers to the Driver Station screen.

/*
 * INTAKE MECHANISM
 *
 * HARDWARE (name must match the robot configuration on the Driver Station):
 *   IntakeMotor = motor that spins the intake
 *
 * HOW IT WORKS:
 *   - The motor turns on at full speed when PLAY is pressed (not during INIT).
 *   - Each press of X flips it: ON (full speed) -> OFF -> ON ...
 */
public class Intake implements Mechanism {

    // ---- Settings ----
    public static double INTAKE_POWER = 1.0; // 1.0 = full speed. Use a negative number to spin backwards.

    // ---- Hardware ----
    // NextMotor doesn't touch the hardwareMap until it's first used, so it's safe to create it here.
    private final NextMotor intakeMotor = new NextMotor("IntakeMotor");

    private boolean on = false; // true while the intake is running.

    // ---- Commands ----
    // instant(...) makes a Command that runs the code inside it once, right away.

    /** Runs the intake at full speed. */
    public Command turnOn() {
        return instant(() -> setOn(true));
    }

    /** Stops the intake. */
    public Command turnOff() {
        return instant(() -> setOn(false));
    }

    /** Flips the intake: ON -> OFF, or OFF -> ON. */
    public Command toggle() {
        return instant(() -> setOn(!on)); // "!" flips true <-> false.
    }

    /**
     * Stops the motor right now (not as a command). Use this in an OpMode's end(),
     * because commands don't get a chance to run after STOP is pressed.
     */
    public void stopNow() {
        on = false;
        intakeMotor.setThrottle(0.0);
        intakeMotor.update(); // Send the 0 power to the motor immediately.
    }

    // Turns the motor on or off and remembers which one it is.
    private void setOn(boolean turnOn) {
        on = turnOn;
        // "a ? b : c" means "if a, use b, otherwise use c".
        intakeMotor.setThrottle(turnOn ? INTAKE_POWER : 0.0);
    }

    // Runs every loop automatically, so we use it to show what the intake is doing.
    @Override
    public void periodic() {
        Telemetry.log("Intake", on ? "ON" : "OFF");
    }
}
