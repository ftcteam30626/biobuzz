package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.telemetry.PanelsTelemetry;        // Sends numbers to Panels and the Driver Station.
import com.bylazar.telemetry.TelemetryManager;       // The object we put telemetry into.
import com.qualcomm.robotcore.hardware.DcMotorEx;    // Motor that can report current.
import com.qualcomm.robotcore.hardware.VoltageSensor; // Reads the battery voltage.
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.robot.opmode.NextOpMode;          // Base class for NextFTC OpModes.
import dev.nextftc.robot.opmode.NextTeleop;          // Puts this OpMode in the Driver Station's TeleOp list.
import dev.nextftc.robot.triggers.CommandGamepad;    // Gamepad that can run Commands when buttons are pressed.
import dev.nextftc.units.Units;                      // Converts shooter speed to RPM.

import org.firstinspires.ftc.teamcode.Mechanisms.Outtake; // Outtake.TARGET_RPM for the target readout.

/*
 * MECHANISMS TELEOP (NextFTC)
 *
 * CONTROLS (gamepad 1):
 *   Left stick  = drive / strafe,  Right stick = turn
 *   Y = shoot (gate opens for 5 seconds, then closes)
 *   A = turn the intake and transfer on / off together
 *   LT = HOLD to align to the hive (right stick is ignored while aligning)
 *
 * PANELS: battery voltage, intake power / voltage / current, shooter RPM and target RPM,
 *         distance to hive (cm), turn power, align mode.
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

    // Panels telemetry, battery voltage, and the intake/transfer motors (for current and power readouts).
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final VoltageSensor batteryVoltage;
    private final DcMotorEx intakeSdk;
    private final DcMotorEx transferSdk;

    // Runs once when INIT is pressed. This is where buttons get hooked up to commands.
    public MechanismsTeleOp(FearOfBot robot) {
        super(robot);       // Let NextFTC set up the robot first.
        this.robot = robot; // Save it so start() and end() can use it.

        robot.limelight.init(hardwareMap); // Starts the Limelight on pipeline 1 for the LT align.

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        intakeSdk = hardwareMap.tryGet(DcMotorEx.class, "IntakeMotor");
        transferSdk = hardwareMap.tryGet(DcMotorEx.class, "TransferMotor");
        robot.transfer.init(hardwareMap); // Transfer's Panels readouts (power and current).

        CommandGamepad driver = new CommandGamepad(gamepad1);

        // onTrue = run the command once, the moment the button is pressed
        driver.x().onTrue(robot.outtake.shoot());
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

        // ---- Panels + Driver Station telemetry ----
        double batteryVolts = batteryVoltage.getVoltage();
        double intakePower = intakeSdk == null ? 0.0 : intakeSdk.getPower();
        panels.addData("Battery voltage (V)", batteryVolts);
        panels.addData("Intake power", intakePower);
        panels.addData("Intake voltage (V, est.)", intakePower * batteryVolts); // Motor sees roughly power x battery.
        panels.addData("Intake current (A)", intakeSdk == null ? 0.0 : intakeSdk.getCurrent(CurrentUnit.AMPS));
        panels.addData("Transfer current (A)", transferSdk == null ? 0.0 : transferSdk.getCurrent(CurrentUnit.AMPS));
        panels.addData("Shooter RPM", robot.outtake.shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute));
        panels.addData("Shooter target RPM", Outtake.TARGET_RPM);
        double distanceCm = robot.limelight.hiveDistanceCm();
        panels.addData("Distance (cm)", Double.isNaN(distanceCm) ? "--" : String.valueOf(Math.round(distanceCm)));
        panels.addData("Turn power", turn);
        panels.addData("Align mode", robot.limelight.mode());
        panels.update(telemetry);
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
