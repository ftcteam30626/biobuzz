package org.firstinspires.ftc.teamcode;

import dev.nextftc.robot.opmode.NextOpMode;          // Base class for NextFTC OpModes.
import dev.nextftc.robot.opmode.NextTeleop;          // Puts this OpMode in the Driver Station's TeleOp list.
import dev.nextftc.robot.triggers.CommandGamepad;    // Gamepad that can run Commands when buttons are pressed.

/*
 * MECHANISMS TELEOP (NextFTC)
 *
 * CONTROLS (gamepad 1):
 *   Left stick  = drive / strafe,  Right stick = turn
 *   B = open / close the shooter gate
 *   X = turn the intake on / off
 *
 * WHAT HAPPENS WHEN:
 *   INIT  = nothing moves.
 *   PLAY  = driving turns on, shooter spins up to Shooter.TARGET_RPM, intake turns on at full speed, gate closes.
 *   STOP  = NextFTC stops everything.
 *
 * NextFTC OpModes use @NextTeleop instead of @TeleOp, and extend NextOpMode.
 * The robot (FearOfBot) is passed in automatically through the constructor.
 */
@NextTeleop(name = "Mechanisms TeleOp", group = "NextFTC")
public class MechanismsTeleOp extends NextOpMode {

    private final FearOfBot robot; // The robot, so we can reach robot.shooter and robot.intake.

    // Runs once when INIT is pressed. This is where buttons get hooked up to commands.
    public MechanismsTeleOp(FearOfBot robot) {
        super(robot);       // Let NextFTC set up the robot first.
        this.robot = robot; // Save it so start() and end() can use it.

        CommandGamepad driver = new CommandGamepad(gamepad1);

        // onTrue = run the command once, the moment the button is pressed
        driver.b().onTrue(robot.shooter.toggleGate());
        driver.x().onTrue(robot.intake.toggle());
    }

    // Runs once when PLAY is pressed (not during INIT).
    // schedule() = "run this command now".
    @Override
    public void start() {
        robot.drivetrain.teleopDrive(gamepad1).schedule(); // Keeps running every loop.
        robot.shooter.closeGate().schedule();
        robot.shooter.spinUp().schedule();
        robot.intake.turnOn().schedule();
    }

    // Runs once when STOP is pressed. The robot object is reused between OpModes,
    // so we turn everything off to start fresh next time.
    @Override
    public void end() {
        robot.drivetrain.stopNow();
        robot.shooter.stopNow();
        robot.intake.stopNow();
    }
}
