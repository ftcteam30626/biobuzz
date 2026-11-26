package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.units.Units;

/*
 * OUTTAKE MECHANISM
 *
 * HARDWARE (names must match the robot configuration on the Driver Station):
 *   ShooterMotor = shooter motor (spins at TARGET_RPM)
 *   GateServo    = servo that blocks/unblocks the balls
 *
 * HOW IT WORKS:
 *   - begin() puts the gate at GATE_START and spins the motor to TARGET_RPM.
 *   - shoot() is the only Command: gate -> GATE_SHOOT, wait GATE_OPEN_MS, gate -> GATE_START.
 */
public class Outtake implements Mechanism {

    // ---- Settings ----
    public static double TARGET_RPM = 2000;   // Speed the shooter motor holds.
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

    /**
     * Sets the gate to GATE_START and spins the shooter motor to TARGET_RPM.
     * Call once when PLAY starts. This is a plain method, not a Command.
     */
    public void begin() {
        gateServo.setPosition(GATE_START);
        shooterMotor.setVelocitySetpoint(Units.getRpm(TARGET_RPM));
    }

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

    // Runs every loop automatically.
    @Override
    public void periodic() {
        shooterMotor.getVelocityConstants().setKP(kP);
        shooterMotor.getVelocityConstants().setKI(kI);
        shooterMotor.getVelocityConstants().setKD(kD);
        shooterMotor.getVelocityConstants().setKV(kF); // NextFTC calls F "kV".

        Telemetry.log("Outtake target RPM", TARGET_RPM);
        Telemetry.log("Outtake RPM", shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute));
    }
}
