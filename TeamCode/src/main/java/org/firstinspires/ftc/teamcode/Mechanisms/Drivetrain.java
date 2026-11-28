package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.telemetry.TelemetryManager;      // The Panels object the robot hands us to fill.
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor; // Reads the battery voltage.

import dev.nextftc.robot.Mechanism;

public class Drivetrain implements Mechanism {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Readings only. Set by init(); null until then.
    private VoltageSensor batteryVoltage;
    private double lastTurn = 0.0; // Turn power sent in the last drive() call.

    /** Finds the motors. Call once from the OpMode's start(), when the hardware map is ready. */
    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
    }

    /** Drives from the gamepad sticks every loop. */
    public Command teleopDrive(Gamepad gamepad) {
        return infinite(() -> {
            double forward = -gamepad.left_stick_y;
            double strafe = gamepad.left_stick_x;
            double turn = gamepad.right_stick_x;
            drive(forward, strafe, turn);
        });
    }

    public void drive(double forward, double strafe, double turn) {
        lastTurn = turn;
        if (leftFront == null) return; // init() has not run yet.

        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn),
                1.0
        );

        double frontLeftPower = (forward + strafe + turn) / denominator;
        double backLeftPower = (forward - strafe + turn) / denominator;
        double frontRightPower = (forward - strafe - turn) / denominator;
        double backRightPower = (forward + strafe - turn) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    public void stopNow() {
        drive(0, 0, 0);
    }

    /** Drive readouts (battery voltage, turn power) for Panels and the Driver Station. */
    public void addTelemetry(TelemetryManager panels) {
        if (batteryVoltage != null) panels.addData("Battery voltage (V)", batteryVoltage.getVoltage());
        panels.addData("Turn power", lastTurn);
    }

    @Override
    public void periodic() {}
}
