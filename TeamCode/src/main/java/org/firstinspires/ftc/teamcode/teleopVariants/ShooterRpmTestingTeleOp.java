package org.firstinspires.ftc.teamcode.teleopVariants;

import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.util.Range;

import com.bylazar.telemetry.PanelsTelemetry;        // Sends numbers to Panels and the Driver Station.
import com.bylazar.telemetry.TelemetryManager;       // The object we put telemetry into.
import com.qualcomm.robotcore.hardware.DcMotorEx;    // Motors that can report current and power.
import com.qualcomm.robotcore.hardware.VoltageSensor; // Reads the battery voltage.
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.teleOp.FearOfBot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.units.Units;

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
 * The shooter RPM only lives in this OpMode. Outtake.TARGET_RPM is not changed.
 */
@NextTeleop(name = "Shooter RPM Testing TeleOp", group = "NextFTC")
public class ShooterRpmTestingTeleOp extends NextOpMode {

    // ---- Settings ----
    private static final double TRIGGER_THRESHOLD = 0.5; // Trigger past this = pressed.
    private static final double RPM_STEP = 10;           // One D-pad press changes the shooter RPM by this much.
    private static final double MAX_RPM = 6000;          // Bare motor top speed.

    private final FearOfBot robot;
    private double shooterRpm = Outtake.TARGET_RPM;      // Shooter speed this OpMode is using.

    // Panels telemetry, battery voltage, and the intake/transfer motors (for current and power readouts).
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final VoltageSensor batteryVoltage;
    private final DcMotorEx intakeSdk;
    private final DcMotorEx transferSdk;

    // Runs once when INIT is pressed. Buttons are hooked up to commands here.
    public ShooterRpmTestingTeleOp(FearOfBot robot) {
        super(robot);
        this.robot = robot;

        robot.limelight.init(hardwareMap); // Starts the Limelight on pipeline 1.

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        intakeSdk = hardwareMap.tryGet(DcMotorEx.class, "IntakeMotor");
        transferSdk = hardwareMap.tryGet(DcMotorEx.class, "TransferMotor");

        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.x().onTrue(robot.outtake.shoot());
        driver.dpadUp().onTrue(Commands.instant(() -> changeRpm(RPM_STEP)));
        driver.dpadDown().onTrue(Commands.instant(() -> changeRpm(-RPM_STEP)));
        driver.y().onTrue(Commands.instant(() -> robot.intake.toggleReverse()));
        driver.b().onTrue(Commands.instant(() -> robot.transfer.toggleReverse()));
    }

    // Runs once when PLAY is pressed.
    @Override
    public void start() {
        robot.drivetrain.init(hardwareMap);
        robot.outtake.begin();             // Gate to rest, shooter to Outtake.TARGET_RPM.
        shooterRpm = Outtake.TARGET_RPM;
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

        // ---- Telemetry ----
        telemetry.addData("Mode", robot.limelight.mode());
        telemetry.addData("Limelight valid", robot.limelight.resultValid());
        telemetry.addData("Pipeline index", robot.limelight.pipelineIndex());
        telemetry.addData("Distance to hive (cm)", show(robot.limelight.hiveDistanceCm(), "%.1f"));
        Telemetry.log("Shooter target RPM", shooterRpm);
        Telemetry.log("Shooter actual RPM", robot.outtake.shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute));

        // ---- Panels telemetry (same readouts as MechanismsTeleOp) ----
        double batteryVolts = batteryVoltage.getVoltage();
        double intakePower = intakeSdk == null ? 0.0 : intakeSdk.getPower();
        panels.addData("Battery voltage (V)", batteryVolts);
        panels.addData("Intake power", intakePower);
        panels.addData("Intake voltage (V, est.)", intakePower * batteryVolts); // Motor sees roughly power x battery.
        panels.addData("Intake current (A)", intakeSdk == null ? 0.0 : intakeSdk.getCurrent(CurrentUnit.AMPS));
        panels.addData("Transfer current (A)", transferSdk == null ? 0.0 : transferSdk.getCurrent(CurrentUnit.AMPS));
        panels.addData("Shooter RPM", robot.outtake.shooterMotor.getEncoderVelocity().into(Units.RotationsPerMinute));
        panels.addData("Shooter target RPM", shooterRpm);
        double distanceCm = robot.limelight.hiveDistanceCm();
        panels.addData("Distance (cm)", Double.isNaN(distanceCm) ? "--" : String.valueOf(Math.round(distanceCm)));
        panels.addData("Turn power", turn);
        panels.addData("Align mode", robot.limelight.mode());
        panels.update(telemetry);
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

    // Changes the shooter speed by 'change' RPM and sends the new speed to the shooter motor.
    private void changeRpm(double change) {
        shooterRpm = Range.clip(shooterRpm + change, 0, MAX_RPM);
        robot.outtake.shooterMotor.setVelocitySetpoint(Units.getRpm(shooterRpm));
    }

    // "NONE" when there is no value (NaN), otherwise the number in the given format.
    private static String show(double value, String format) {
        return Double.isNaN(value) ? "NONE" : String.format(format, value);
    }
}
