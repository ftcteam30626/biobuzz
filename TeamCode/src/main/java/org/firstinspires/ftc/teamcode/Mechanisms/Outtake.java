package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.telemetry.TelemetryManager;   // The Panels object the robot hands us to fill.
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.util.Range;        // Keeps the shooter speed between 0 and MAX_RPM.

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/*
 * OUTTAKE MECHANISM
 *
 * HARDWARE (names must match the robot configuration on the Driver Station):
 *   ShooterMotor = shooter motor (spins at targetRpm)
 *   GateServo    = servo that blocks/unblocks the balls
 *
 * HOW IT WORKS:
 *   - begin() puts the gate at GATE_START and spins the motor to TARGET_RPM.
 *   - changeTargetRpm() speeds the shooter up or down while it runs (used by the tester OpModes).
 *   - shoot() is the only Command: gate -> GATE_SHOOT, wait GATE_OPEN_MS, gate -> GATE_START.
 *   - addTelemetry(panels) adds this mechanism's readouts. FearOfBot.sendTelemetry() calls it.
 */
public class Outtake implements Mechanism {

    // ---- Settings ----
    public static double TARGET_RPM = 2000;   // Speed the shooter starts at (begin() resets to this).
    public static double MAX_RPM = 6000;      // Highest speed changeTargetRpm() allows. Bare motor top speed.
    public static double GATE_START = 0.75;    // Gate position at rest.
    public static double GATE_SHOOT = 0.5;    // Gate position while shooting.
    public static long GATE_OPEN_MS = 3000;   // How long the gate stays at GATE_SHOOT.

    // ---- PIDF gains (copied from the Shooter tuning; retune if needed) ----
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00025;

    // Assumes goBILDA bare motors (28 encoder ticks per turn). Change if the motors differ.
    private static final double TICKS_PER_REV = 28.0;

    // ---- Hardware ----
    public final NextMotor shooterMotor =
            new NextMotor("ShooterMotor", Units.getRotations(1.0 / TICKS_PER_REV));
    // -1 turns off NextFTC's "skip if same position" caching, so the gate is always re-sent its position.
    private final NextServo gateServo = new NextServo("GateServo", -1.0);

    // The speed the shooter is commanded to hold right now. Not final: changeTargetRpm() moves it.
    private double targetRpm = TARGET_RPM;

    /**
     * Sets the gate to GATE_START and spins the shooter motor to TARGET_RPM.
     * Call once when PLAY starts. This is a plain method, not a Command.
     */
    public void begin() {
        gateServo.setPosition(GATE_START);
        targetRpm = TARGET_RPM;
        shooterMotor.setVelocitySetpoint(Units.getRpm(targetRpm));
    }

    /** Adds 'change' RPM (negative to slow down) to the shooter speed, keeps it in 0..MAX_RPM, and sends it to the motor. */
    public void changeTargetRpm(double change) {
        targetRpm = Range.clip(targetRpm + change, 0, MAX_RPM);
        shooterMotor.setVelocitySetpoint(Units.getRpm(targetRpm));
    }

    /** The shooter speed currently commanded, in RPM. */
    public double targetRpm() { return targetRpm; }

    /** Gate to GATE_SHOOT, wait GATE_OPEN_MS, then gate back to GATE_START. */
    public Command shoot() {
        return instant(() -> gateServo.setPosition(GATE_SHOOT))
                .then(Commands.waitMs(GATE_OPEN_MS))
                .then(instant(() -> gateServo.setPosition(GATE_START)));
    }

    /** Stops the shooter motor right now. Use this in an OpMode's end(). */
    public void stopNow() {
        shooterMotor.setThrottle(0.0);
        shooterMotor.update();
    }

    /** The shooter speed the encoder reports right now, in RPM. */
    public double actualRpm() {
        return shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute);
    }

    /** Shooter readouts (target and actual RPM) for Panels and the Driver Station. */
    public void addTelemetry(TelemetryManager panels) {
        panels.addData("Shooter target RPM", targetRpm);
        panels.addData("Shooter RPM", actualRpm());
    }

    // Runs every loop automatically.
    @Override
    public void periodic() {
        shooterMotor.getVelocityConstants().setKP(kP);
        shooterMotor.getVelocityConstants().setKI(kI);
        shooterMotor.getVelocityConstants().setKD(kD);
        shooterMotor.getVelocityConstants().setKV(kF); // NextFTC calls F "kV".
    }
}
