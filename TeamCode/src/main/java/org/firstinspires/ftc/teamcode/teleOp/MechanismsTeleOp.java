package org.firstinspires.ftc.teamcode.teleOp;

import dev.nextftc.robot.opmode.NextOpMode;          // Base class for NextFTC OpModes.
import dev.nextftc.robot.opmode.NextTeleop;          // Puts this OpMode in the Driver Station's TeleOp list.
import dev.nextftc.robot.triggers.CommandGamepad;    // Gamepad that can run Commands when buttons are pressed.

/*
 * MECHANISMS TELEOP (NextFTC)
 *
 * CONTROLS (gamepad 1):
 *   Left stick  = drive / strafe,  Right stick = turn
 *   Y = shoot (gate opens for 5 seconds, then closes)
 *   A = turn the intake and transfer on / off together
 *   LT = HOLD to align to the hive (right stick is ignored while aligning)
 *
 * WHAT HAPPENS WHEN:
 *   INIT  = nothing moves.
 *   PLAY  = driving turns on, shooter spins up to Outtake.TARGET_RPM, intake and transfer turn on, gate goes to GATE_START.
 *   STOP  = NextFTC stops everything.
 *
 * NextFTC OpModes use @NextTeleop instead of @TeleOp, and extend NextOpMode.
 * The robot (FearOfBot) is passed in automatically through the constructor.
 */
@NextTeleop(name = "Mechanisms TeleOp", group = "NextFTC")
public class MechanismsTeleOp extends NextOpMode {

    private final FearOfBot robot; // The robot, so we can reach robot.outtake and robot.intake.

    // Runs once when INIT is pressed. This is where buttons get hooked up to commands.
    public MechanismsTeleOp(FearOfBot robot) {
        super(robot);       // Let NextFTC set up the robot first.
        this.robot = robot; // Save it so start() and end() can use it.

        robot.limelight.init(hardwareMap); // Starts the Limelight on pipeline 1 for the LT align.

        CommandGamepad driver = new CommandGamepad(gamepad1);

        // onTrue = run the command once, the moment the button is pressed
        driver.x().onTrue(robot.outtake.shoot());
        driver.a().onTrue(robot.intake.toggleWith(robot.transfer));
    }

    // Runs once when PLAY is pressed (not during INIT).
    // schedule() = "run this command now".
    @Override
    public void start() {
        robot.drivetrain.init(hardwareMap); // Find the drive motors (hardware map is ready now).
        robot.outtake.begin(); // Gate to GATE_START, motors to TARGET_RPM.
        robot.intake.turnOn().schedule();
        robot.transfer.setOn(true);
    }

    // Runs every loop while PLAY is active. Drives from the sticks; LT holds the align.
    @Override
    public void periodic() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        boolean aligning = gamepad1.left_trigger > 0.5;
        double turn = robot.limelight.turnPower(aligning, gamepad1.right_stick_x);
        robot.drivetrain.drive(forward, strafe, turn);
    }

    // Runs once when STOP is pressed. The robot object is reused between OpModes,
    // so we turn everything off to start fresh next time.
    @Override
    public void end() {
        robot.drivetrain.stopNow();
        robot.outtake.stopNow();
        robot.intake.stopNow();
        robot.transfer.stopNow();
        robot.limelight.stopNow();
    }
}
