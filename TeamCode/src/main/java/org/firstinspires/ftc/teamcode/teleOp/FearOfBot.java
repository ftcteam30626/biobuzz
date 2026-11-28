package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.telemetry.PanelsTelemetry;   // Panels: the laptop screen for numbers and live tuning.
import com.bylazar.telemetry.TelemetryManager;  // The object every mechanism adds its readouts to.
import org.firstinspires.ftc.robotcore.external.Telemetry; // The Driver Station screen.

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Limelight;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

/*
 * THE ROBOT
 *
 * This class holds every mechanism on the robot. NextFTC finds it automatically
 * (because it implements NextRobot) and hands it to every NextFTC OpMode.
 * Mechanisms listed in getMechanisms() get their periodic() called every loop.
 *
 * TELEMETRY:
 *   Each mechanism has an addTelemetry(panels) method with ONLY its own readouts.
 *   The robot owns the one Panels manager (panels), and sendTelemetry() gathers every
 *   mechanism's lines and sends them to Panels and the Driver Station. OpModes call
 *   robot.sendTelemetry(telemetry) once per loop, at the end of periodic().
 *
 * To add a new mechanism: make it in the Mechanisms folder, add a field here,
 * add it to getMechanisms(), and add it to sendTelemetry().
 */
public class FearOfBot implements NextRobot {
    // One of each mechanism. "public final" = OpModes can use them, but can't swap them out.
    public final Drivetrain drivetrain = new Drivetrain();
    public final Outtake outtake = new Outtake();
    public final Intake intake = new Intake();
    public final Transfer transfer = new Transfer();
    public final Limelight limelight = new Limelight();

    // The one Panels manager for the whole robot. OpModes can add their own lines to it too.
    public final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    // Tells NextFTC which mechanisms to run periodic() on every loop.
    @Override
    public Set<Mechanism> getMechanisms() {
        // Set.of(...) would be shorter, but it crashes on the Control Hub's older Android version.
        return new HashSet<>(Arrays.asList(drivetrain, outtake, intake, transfer, limelight));
    }

    /**
     * Sends every mechanism's readouts to Panels and the Driver Station. Call once per loop,
     * at the end of an OpMode's periodic(). A mechanism that was not set up in this OpMode adds nothing.
     */
    public void sendTelemetry(Telemetry telemetry) {
        drivetrain.addTelemetry(panels);
        outtake.addTelemetry(panels);
        intake.addTelemetry(panels);
        transfer.addTelemetry(panels);
        limelight.addTelemetry(panels);
        panels.update(telemetry);
    }
}
