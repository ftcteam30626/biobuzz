package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.telemetry.TelemetryManager;   // The Panels object the robot hands us to fill.
import com.pedropathing.ivy.Command;              // A small action the robot can run (e.g. "turn on").
import com.qualcomm.robotcore.hardware.DcMotorEx; // SDK motor, used to read power and current.
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor; // Reads the battery voltage.
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.hardware.actuators.NextMotor;  // NextFTC's motor.
import dev.nextftc.robot.Mechanism;               // Marks this class as one part of the robot.

/*
 * INTAKE MECHANISM
 *
 * HARDWARE (name must match the robot configuration on the Driver Station):
 *   IntakeMotor = motor that spins the intake
 *
 * HOW IT WORKS:
 *   - The motor turns on at full speed when PLAY is pressed (not during INIT).
 *   - toggleWith(transfer) flips the intake and the transfer together (A button).
 *   - addTelemetry(panels) adds this mechanism's readouts. FearOfBot.sendTelemetry() calls it.
 */
public class Intake implements Mechanism {

    // ---- Settings ----
    public static double INTAKE_POWER = 1.0; // 1.0 = full speed. Use a negative number to spin backwards.

    // ---- Hardware ----
    // NextMotor doesn't touch the hardwareMap until it's first used, so it's safe to create it here.
    private final NextMotor intakeMotor = new NextMotor("IntakeMotor");

    // Readings only (power, current, battery). Set by init(); null until then.
    private DcMotorEx sdkMotor;
    private VoltageSensor batteryVoltage;

    private boolean on = false; // true while the intake is running.

    private boolean reversed;

    public Intake() {
        setReversed(true); // Starts reversed.
    }

    /** Flips the intake motor between REVERSE and FORWARD. A plain method, not a Command. */
    public void toggleReverse() {
        setReversed(!reversed);
    }

    private void setReversed(boolean reverse) {
        reversed = reverse;
        intakeMotor.setDirection(reverse ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);
    }

    /** Finds the motor and battery for readings. Call once from the OpMode constructor. */
    public void init(HardwareMap hardwareMap) {
        sdkMotor = hardwareMap.tryGet(DcMotorEx.class, "IntakeMotor");
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }

    // ---- Commands ----
    // instant(...) makes a Command that runs the code inside it once, right away.

    /** Runs the intake at full speed. */
    public Command turnOn() {
        return instant(() -> setOn(true));
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

    /** Intake readouts (ON/OFF, power, estimated voltage, current) for Panels and the Driver Station. */
    public void addTelemetry(TelemetryManager panels) {
        panels.addData("Intake", on ? "ON" : "OFF");
        if (sdkMotor == null) return; // init() has not run in this OpMode.
        double power = sdkMotor.getPower();
        panels.addData("Intake power", power);
        panels.addData("Intake voltage (V, est.)", power * batteryVoltage.getVoltage()); // Motor sees roughly power x battery.
        panels.addData("Intake current (A)", sdkMotor.getCurrent(CurrentUnit.AMPS));
    }

    @Override
    public void periodic() {}
}
