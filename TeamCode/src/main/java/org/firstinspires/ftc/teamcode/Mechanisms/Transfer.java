package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.telemetry.PanelsTelemetry;        // Sends numbers to Panels.
import com.bylazar.telemetry.TelemetryManager;       // The object we put telemetry into.
import com.qualcomm.robotcore.hardware.DcMotorEx;    // SDK motor, used to read power and current.
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

/*
 * TRANSFER MECHANISM
 *
 * HARDWARE (name must match the robot configuration on the Driver Station):
 *   TransferMotor = motor that moves balls from the intake to the shooter
 *
 * HOW IT WORKS:
 *   - Turned on and off by setOn(). Intake.toggleWith(transfer) calls it together with the intake.
 */
public class Transfer implements Mechanism {

    // ---- Settings ----
    public static double TRANSFER_POWER = 0.75; // 1.0 = full speed.

    // ---- Hardware ----
    public final NextMotor transferMotor = new NextMotor("TransferMotor");

    private boolean reversed;

    public Transfer() {
        setReversed(false); // Starts forward.
    }

    /** Flips the transfer motor between FORWARD and REVERSE. A plain method, not a Command. */
    public void toggleReverse() {
        setReversed(!reversed);
    }

    private void setReversed(boolean reverse) {
        reversed = reverse;
        transferMotor.setDirection(reverse ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);
    }

    // Panels readouts (power and current). The OpMode calls panels.update(telemetry) to send them.
    private TelemetryManager panels;
    private DcMotorEx sdkMotor; // Same "TransferMotor" as transferMotor, used only for power and current readings.

    /** Finds the motor for readings. Call once from the OpMode constructor. */
    public void init(HardwareMap hardwareMap) {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        sdkMotor = hardwareMap.tryGet(DcMotorEx.class, "TransferMotor");
    }

    // Runs every loop automatically: adds the transfer's power and current to Panels.
    @Override
    public void periodic() {
        if (sdkMotor == null || panels == null) return;
        panels.addData("Transfer power", sdkMotor.getPower());
        panels.addData("Transfer current (A)", sdkMotor.getCurrent(CurrentUnit.AMPS));
    }

    /** Turns the transfer on at TRANSFER_POWER, or off. A plain method, not a Command. */
    public void setOn(boolean on) {
        transferMotor.setThrottle(on ? TRANSFER_POWER : 0.0);
    }

    /** Stops the motor right now. Use this in an OpMode's end(). */
    public void stopNow() {
        transferMotor.setThrottle(0.0);
        transferMotor.update();
    }
}
