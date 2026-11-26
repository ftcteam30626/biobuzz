package org.firstinspires.ftc.teamcode.Mechanisms;

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
    public static double TRANSFER_POWER = 1.0; // 1.0 = full speed.

    // ---- Hardware ----
    public final NextMotor transferMotor = new NextMotor("TransferMotor");

    public Transfer() {
        transferMotor.setDirection(NextMotor.Direction.REVERSE);
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
