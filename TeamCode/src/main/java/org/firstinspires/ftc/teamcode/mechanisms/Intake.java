package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;

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
    public static double INTAKE_POWER = 1.0;

    // ---- Hardware ----
    private final NextMotor intakeMotor = new NextMotor("IntakeMotor");

    private boolean on = false;

    // ---- Commands ----

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
        return instant(() -> setOn(!on));
    }

    /**
     * Stops the motor right now (not as a command). Use this in an OpMode's end(),
     * because commands don't get a chance to run after STOP is pressed.
     */
    public void stopNow() {
        on = false;
        intakeMotor.setThrottle(0.0);
        intakeMotor.update();
    }

    private void setOn(boolean turnOn) {
        on = turnOn;
        intakeMotor.setThrottle(turnOn ? INTAKE_POWER : 0.0);
    }

    // Runs every loop automatically, so we use it to show what the intake is doing.
    @Override
    public void periodic() {
        Telemetry.log("Intake", on ? "ON" : "OFF");
    }
}
