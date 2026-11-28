package org.firstinspires.ftc.teamcode.teleopVariants;

import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

import org.firstinspires.ftc.teamcode.teleOp.FearOfBot;

/*
 * SHOOTER RPM TESTING TELEOP (NextFTC)
 *
 * CONTROLS (gamepad 1):
 *   Left stick     = drive / strafe
 *   Right stick    = turn (only when NOT aligning)
 *   LEFT TRIGGER   = HOLD to align to the hive (uses the Limelight mechanism)
 *   RIGHT TRIGGER  = shoot (press once; gate opens for GATE_OPEN_MS)
 *   A              = intake + transfer on / off
 *   D-PAD UP       = shooter RPM +RPM_STEP
 *   D-PAD DOWN     = shooter RPM -RPM_STEP
 *   Y              = flip intake motor direction (starts REVERSE)
 *   B              = flip transfer motor direction (starts FORWARD)
 *
 * The shooter RPM change only lives in this OpMode (kept by Outtake until the next PLAY).
 * Outtake.TARGET_RPM is not changed.
 * TELEMETRY: every mechanism shows its own readouts, sent by robot.sendTelemetry().
 */
@NextTeleop(name = "Shooter RPM Testing TeleOp", group = "NextFTC")
public class ShooterRpmTestingTeleOp extends NextOpMode {

    // ---- Settings ----
    private static final double TRIGGER_THRESHOLD = 0.5; // Trigger past this = pressed.
    private static final double RPM_STEP = 10;           // One D-pad press changes the shooter RPM by this much.

    private final FearOfBot robot;

    // Runs once when INIT is pressed. Buttons are hooked up to commands here.
    public ShooterRpmTestingTeleOp(FearOfBot robot) {
        super(robot);
        this.robot = robot;

        robot.limelight.init(hardwareMap); // Starts the Limelight on pipeline 1.
        robot.intake.init(hardwareMap);    // Readings (power, current, voltage).
        robot.transfer.init(hardwareMap);  // Readings (power, current).

        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.x().onTrue(robot.outtake.shoot());
        driver.dpadUp().onTrue(Commands.instant(() -> robot.outtake.changeTargetRpm(RPM_STEP)));
        driver.dpadDown().onTrue(Commands.instant(() -> robot.outtake.changeTargetRpm(-RPM_STEP)));
        driver.y().onTrue(Commands.instant(() -> robot.intake.toggleReverse()));
        driver.b().onTrue(Commands.instant(() -> robot.transfer.toggleReverse()));
    }

    // Runs once when PLAY is pressed.
    @Override
    public void start() {
        robot.drivetrain.init(hardwareMap);
        robot.outtake.begin();             // Gate to rest, shooter to Outtake.TARGET_RPM.
        robot.intake.turnOn().schedule();
        robot.transfer.setOn(true);
    }

    // Runs every loop while PLAY is active.
    @Override
    public void periodic() {
        // ---- Drive. Hold LT to align; the right stick is ignored while aligning. ----
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        boolean aligning = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        double turn = robot.limelight.turnPower(aligning, gamepad1.right_stick_x);
        robot.drivetrain.drive(forward, strafe, turn);

        robot.sendTelemetry(telemetry);
    }

    // Runs once when STOP is pressed.
    @Override
    public void end() {
        robot.drivetrain.stopNow();
        robot.outtake.stopNow();
        robot.intake.stopNow();
        robot.transfer.stopNow();
        robot.limelight.stopNow();
    }
}
