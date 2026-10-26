package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/*
 * BARE MOTOR RPM TESTER (goBILDA Bare motor named "motor")
 *
 * This is a TeleOp OpMode. On the Driver Station it is found in the TeleOp list.
 *
 * HOW TO USE:
 *   1. Make sure your robot configuration has a motor named "motor".
 *   2. Select the TeleOp "Bare Motor RPM Tester" and press START. The motor spins up to 1000 RPM.
 *   3. Gamepad 1 D-pad UP = +50 RPM, D-pad DOWN = -50 RPM.
 *   4. When the motor is within 30 RPM of the target, the screen says "At target RPM".
 */
@Utility(name = "Bare Motor RPM Tester")
public class BareMotorRPMTester extends OpMode {

    // ---- Settings ----
    public static String MOTOR_NAME = "motor";
    public static double START_RPM = 1000;
    public static double RPM_STEP = 50;
    public static double TOLERANCE_RPM = 30;  // How close counts as "at target" (+/- 30 RPM).
    public static double MAX_RPM = 6000;      // Bare motor top speed.

    private static final double TICKS_PER_REV = 28.0;

    private DcMotorEx motor;
    private double targetRpm = START_RPM;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastB = false;
    private boolean reversed = false;         // false = forward, true = reverse.

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        // RUN_USING_ENCODER lets the hub hold the speed we ask for on its own.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coast to a stop.
        telemetry.addLine("Ready. Press START to spin up to " + (int) START_RPM + " RPM.");
    }

    @Override
    public void start() {
        targetRpm = START_RPM;
        reversed = false;
    }

    @Override
    public void loop() {
        // Only change once per press (not every loop while the button is held).
        if (gamepad1.dpad_up && !lastUp) targetRpm += RPM_STEP;
        if (gamepad1.dpad_down && !lastDown) targetRpm -= RPM_STEP;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        if (gamepad1.b && !lastB) reversed = !reversed;
        lastB = gamepad1.b;
        targetRpm = Range.clip(targetRpm, 0, MAX_RPM);

        // Signed target: positive = forward, negative = reverse.
        double signedTargetRpm = reversed ? -targetRpm : targetRpm;

        // The motor works in ticks per second, so convert RPM to and from that.
        motor.setVelocity(signedTargetRpm * TICKS_PER_REV / 60.0);
        double currentRpm = motor.getVelocity() / TICKS_PER_REV * 60.0;

        boolean atTarget = targetRpm > 0 && Math.abs(signedTargetRpm - currentRpm) <= TOLERANCE_RPM;

        telemetry.addLine("D-pad UP = +" + (int) RPM_STEP + " RPM | D-pad DOWN = -" + (int) RPM_STEP + " RPM | B = flip direction");
        telemetry.addData("Direction", reversed ? "REVERSE" : "FORWARD");
        telemetry.addData("Target RPM", "%.0f", signedTargetRpm);
        telemetry.addData("Current RPM", "%.0f", currentRpm);
        telemetry.addData("Error RPM", "%.0f", signedTargetRpm - currentRpm);
        telemetry.addLine(targetRpm == 0 ? "Stopped" : atTarget ? "At target RPM" : "Adjusting...");
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
