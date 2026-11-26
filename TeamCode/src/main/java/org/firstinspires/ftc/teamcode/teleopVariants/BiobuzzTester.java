package org.firstinspires.ftc.teamcode.teleopVariants;

import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.units.Units;

import org.firstinspires.ftc.teamcode.teleOp.FearOfBot;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;

/*
 * BIOBUZZ TESTER
 *
 * Same controls and setup as MechanismsTeleOp, with one addition:
 *   Gamepad 1 RIGHT BUMPER = shooter RPM +RPM_STEP
 *   Gamepad 1 LEFT BUMPER  = shooter RPM -RPM_STEP
 *
 * The RPM change only lives in this OpMode. Outtake.TARGET_RPM is not changed,
 * so MechanismsTeleOp still starts at its own TARGET_RPM.
 */
@NextTeleop(name = "Biobuzz Tester", group = "Utilities")
public class BiobuzzTester extends NextOpMode {

    // ---- Settings ----
    public static double RPM_STEP = 50; // How much one D-pad press changes the shooter speed.

    private final FearOfBot robot;
    private double targetRpm = Outtake.TARGET_RPM; // The shooter speed this OpMode is currently using.

    public BiobuzzTester(FearOfBot robot) {
        super(robot);
        this.robot = robot;

        CommandGamepad driver = new CommandGamepad(gamepad1);

        driver.x().onTrue(robot.outtake.shoot());
        driver.a().onTrue(robot.intake.toggleWith(robot.transfer));
        driver.rightBumper().onTrue(Commands.instant(() -> changeRpm(RPM_STEP)));
        driver.leftBumper().onTrue(Commands.instant(() -> changeRpm(-RPM_STEP)));
    }

    @Override
    public void start() {
        targetRpm = Outtake.TARGET_RPM;
        robot.drivetrain.init(hardwareMap);
        robot.drivetrain.teleopDrive(gamepad1).schedule();
        robot.outtake.begin();
        robot.intake.turnOn().schedule();
        robot.transfer.setOn(true);

        // Runs every loop: updates the shooter RPM display.
        Commands.infinite(this::showShooterRpm).schedule();
    }

    // Shows the shooter RPM on the Driver Station every loop.
    private void showShooterRpm() {
        // Target = what we asked for. Actual = what the motor's encoder reports now.
        Telemetry.log("Shooter target RPM", targetRpm);
        Telemetry.log("Shooter actual RPM", robot.outtake.shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute));
    }

    @Override
    public void end() {
        robot.drivetrain.stopNow();
        robot.outtake.stopNow();
        robot.intake.stopNow();
        robot.transfer.stopNow();
    }

    // Changes the shooter speed by 'change' RPM and sends the new speed to the motor.
    private void changeRpm(double change) {
        targetRpm = Math.max(0, targetRpm + change);
        robot.outtake.shooterMotor.setVelocitySetpoint(Units.getRpm(targetRpm));
    }
}
