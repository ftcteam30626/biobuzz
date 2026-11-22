package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * MULTI MOTOR RPM TESTER
 *
 * Switch between 7 motors. Only the selected motor spins, at full power.
 *
 * HOW TO USE:
 *   1. Make sure the robot configuration has all 7 motors with these names:
 *      leftFront, leftBack, rightBack, rightFront, TransferMotor, ShooterMotor, IntakeMotor
 *   2. Select the TeleOp "Multi Motor RPM Tester" and press START.
 *   3. Gamepad 1 LEFT BUMPER / RIGHT BUMPER = switch motor.
 *   4. Gamepad 1 D-pad UP = selected motor full speed forward.
 *      Gamepad 1 D-pad DOWN = selected motor full speed backward.
 *      Let go of the D-pad to stop.
 */
@Utility(name = "Multi Motor RPM Tester")
public class MultiMotorRPMTester extends OpMode {

    // ---- Settings ----
    private static final String[] MOTOR_NAMES = {
            "leftFront", "leftBack", "rightBack", "rightFront",
            "TransferMotor", "ShooterMotor", "IntakeMotor"
    };

    // ---- State ----
    private DcMotor[] motors;                  // null if the motor was not found in the config.
    private int selected = 0;                  // Index of the motor being tested.

    // Last loop's bumper states, so a held bumper only switches once.
    private boolean lastLeft = false;
    private boolean lastRight = false;

    // Runs once when INIT is pressed.
    @Override
    public void init() {
        int count = MOTOR_NAMES.length;
        motors = new DcMotor[count];

        for (int i = 0; i < count; i++) {
            try {
                motors[i] = hardwareMap.get(DcMotor.class, MOTOR_NAMES[i]);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motors[i].setPower(0);
            } catch (IllegalArgumentException e) {
                motors[i] = null; // Not in the robot configuration.
            }
        }
        telemetry.addLine("Ready. Press START. Use LEFT/RIGHT BUMPER to pick a motor.");
    }

    // Runs over and over while the OpMode is playing.
    @Override
    public void loop() {
        int count = MOTOR_NAMES.length;

        // Switch motor (once per press).
        if (gamepad1.left_bumper && !lastLeft) selected = (selected + count - 1) % count;
        if (gamepad1.right_bumper && !lastRight) selected = (selected + 1) % count;
        lastLeft = gamepad1.left_bumper;
        lastRight = gamepad1.right_bumper;

        // D-pad UP = full forward, D-pad DOWN = full backward, otherwise stop.
        double power = 0;
        if (gamepad1.dpad_up) power = -0.8;
        else if (gamepad1.dpad_down) power = 0.8;

        // Only the selected motor gets power. Every other motor is stopped.
        for (int i = 0; i < count; i++) {
            if (motors[i] != null) {
                motors[i].setPower(i == selected ? power : 0);
            }
        }

        telemetry.addLine("LEFT/RIGHT BUMPER = switch motor | D-PAD UP = full forward | D-PAD DOWN = full backward");
        telemetry.addData("Selected", (selected + 1) + "/" + count + ": " + MOTOR_NAMES[selected]);

        if (motors[selected] == null) {
            telemetry.addLine("NOT FOUND in robot configuration");
        } else {
            telemetry.addData("Power", "%.1f", power);
        }

        // Show which motors were found.
        for (int i = 0; i < count; i++) {
            telemetry.addLine((i == selected ? "> " : "  ") + MOTOR_NAMES[i]
                    + (motors[i] == null ? " (NOT FOUND)" : ""));
        }
        telemetry.update();
    }

    // Runs once when STOP is pressed.
    @Override
    public void stop() {
        if (motors == null) return;
        for (DcMotor motor : motors) {
            if (motor != null) motor.setPower(0);
        }
    }
}
