package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * PANELS PIDF TUNER (bare motor, speed in RPM)
 *
 * HOW TO USE:
 *   1. INIT, then PLAY.
 *   2. Panels > Graph: add "target RPM" and "actual RPM".
 *   3. Gamepad 1: A = start motor, B = stop motor.
 *   4. Tune F first, then P. Only add I or D if you need them.
 *   5. Copy the final numbers into Shooter.java (kP, kI, kD, kF).
 *      Edit values live in Panels!
 */
@TeleOp(name = "Panels PIDF Tuner")
public class PanelsPIDFTuner extends OpMode {

    private static final double TICKS_PER_REV = 28.0;

    // ===== Remembered between loops =====
    private DcMotorEx motor;
    private TelemetryManager panels;
    private double integralSum = 0;
    private double lastError = 0;
    private boolean running = false;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        motor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // our code does the PIDF
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        motor.setDirection(PIDFConfig.REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        if (gamepad1.a) running = true;   // A = start motor
        if (gamepad1.b) running = false;  // B = stop motor

        double actualRpm = motor.getVelocity() / TICKS_PER_REV * 60.0;
        double targetRpm = running ? PIDFConfig.TARGET_RPM : 0;
        double power = 0;

        double dt = timer.seconds();
        timer.reset();

        if (running) {
            double error = targetRpm - actualRpm;
            integralSum += error * dt;
            double change = (error - lastError) / dt;
            lastError = error;

            power = PIDFConfig.P * error + PIDFConfig.I * integralSum + PIDFConfig.D * change + PIDFConfig.F * targetRpm;
            power = Range.clip(power, -1, 1);
        } else {
            integralSum = 0;
            lastError = 0;
        }

        motor.setPower(power);

        // ===== Graph these =====
        panels.addData("target RPM", targetRpm);
        panels.addData("actual RPM", actualRpm);
        panels.addData("error RPM", targetRpm - actualRpm);
        panels.addData("power", power);

        // ===== Values the robot is actually using (check your Panels edits arrived) =====
        panels.addData("TARGET_RPM", PIDFConfig.TARGET_RPM);
        panels.addData("P", PIDFConfig.P);
        panels.addData("I", PIDFConfig.I);
        panels.addData("D", PIDFConfig.D);
        panels.addData("F", PIDFConfig.F);
        panels.addData("REVERSED", PIDFConfig.REVERSED);

        // ===== What each letter does =====
        panels.addLine("");
        panels.addLine("A = start motor | B = stop motor");
        panels.addLine("F = Base power");
        panels.addLine("P = Catch up (big gaps)");
        panels.addLine("I = Close the last gap (small gaps that stay)");
        panels.addLine("D = Brake (stops going past target)");

        // ===== How to tune =====
        panels.addLine("");
        panels.addLine("Edit in Panels > change values live");
        panels.addLine("1. F: below target -> raise | above -> lower");
        panels.addLine("2. P: takes long to reach -> raise | bounces -> lower");
        panels.addLine("3. Still under -> add tiny I | goes past -> add tiny D");
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}
