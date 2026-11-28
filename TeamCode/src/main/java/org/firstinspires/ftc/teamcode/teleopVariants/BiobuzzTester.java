package org.firstinspires.ftc.teamcode.teleopVariants;

import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

import org.firstinspires.ftc.teamcode.teleOp.FearOfBot;

/*
 * BIOBUZZ TESTER
 *
 * Same controls and setup as MechanismsTeleOp, with one addition:
 *   Gamepad 1 RIGHT BUMPER = shooter RPM +RPM_STEP
 *   Gamepad 1 LEFT BUMPER  = shooter RPM -RPM_STEP
 *
 * The RPM change only lives in this OpMode. Outtake.TARGET_RPM is not changed,
 * so MechanismsTeleOp still starts at its own TARGET_RPM.
 * Telemetry comes from the mechanisms via robot.sendTelemetry().
 */
@NextTeleop(name = "Biobuzz Tester")
public class BiobuzzTester extends NextOpMode {

    // ---- Settings ----
    public static double RPM_STEP = 50; // How much one bumper press changes the shooter speed.

    private final FearOfBot robot;

    public BiobuzzTester(FearOfBot robot) {
        super(robot);
        this.robot = robot;

        robot.intake.init(hardwareMap);   // Readings (power, current, voltage).
        robot.transfer.init(hardwareMap); // Readings (power, current).

        CommandGamepad driver = new CommandGamepad(gamepad1);

        driver.x().onTrue(robot.outtake.shoot());
        driver.rightBumper().onTrue(Commands.instant(() -> robot.outtake.changeTargetRpm(RPM_STEP)));
        driver.leftBumper().onTrue(Commands.instant(() -> robot.outtake.changeTargetRpm(-RPM_STEP)));
    }

    @Override
    public void start() {
        robot.drivetrain.init(hardwareMap);
        robot.drivetrain.teleopDrive(gamepad1).schedule();
        robot.outtake.begin();
        robot.intake.turnOn().schedule();
        robot.transfer.setOn(true);
    }

    // Runs every loop: sends every mechanism's readouts to Panels and the Driver Station.
    @Override
    public void periodic() {
        robot.sendTelemetry(telemetry);
    }

    @Override
    public void end() {
        robot.drivetrain.stopNow();
        robot.outtake.stopNow();
        robot.intake.stopNow();
        robot.transfer.stopNow();
    }
}
