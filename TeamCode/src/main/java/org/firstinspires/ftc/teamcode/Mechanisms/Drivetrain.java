package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Drivetrain implements Mechanism {

    private final NextMotor leftFront = new NextMotor("leftFront");
    private final NextMotor rightFront = new NextMotor("rightFront");
    private final NextMotor rightBack = new NextMotor("rightBack");
    private final NextMotor leftBack = new NextMotor("leftBack");
    private final NextMotor[] motors = {leftFront, rightFront, rightBack, leftBack};

    private final double[] currentPower = new double[4];
    private final double[] holdTarget = new double[4];
    private boolean holding = false;
    private final ElapsedTime loopTimer = new ElapsedTime();

    public Drivetrain() {
        for (NextMotor motor : motors) motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    public Command teleopDrive(Gamepad gamepad) {
        return infinite(() -> {
            double drive = shape(-gamepad.left_stick_y);
            double strafe = shape(gamepad.left_stick_x);
            double turn = gamepad.right_stick_x > 0.05 || gamepad.right_stick_x < -0.05 ? gamepad.right_stick_x : 0;
            drive(drive, strafe, turn);
        });
    }

    public void drive(double drive, double strafe, double turn) {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0.1) dt = 0.02;

        if (drive == 0 && strafe == 0 && turn == 0) {
            holdPosition();
        } else {
            holding = false;
            double[] target = {
                    drive + strafe + turn,
                    drive - strafe - turn,
                    drive + strafe - turn,
                    drive - strafe + turn
            };

            double biggest = 1.0;
            for (double p : target) biggest = Math.max(biggest, Math.abs(p));
            for (int i = 0; i < 4; i++) {
                target[i] = target[i] / biggest;
                boolean speedingUp = Math.abs(target[i]) > Math.abs(currentPower[i]) || Math.signum(target[i]) != Math.signum(currentPower[i]);
                double maxStep = (speedingUp ? 3.0 : 10.0) * dt;
                currentPower[i] += Range.clip(target[i] - currentPower[i], -maxStep, maxStep);
                motors[i].setThrottle(currentPower[i]);
            }
        }
    }

    private void holdPosition() {
        if (!holding) {
            for (int i = 0; i < 4; i++) {
                holdTarget[i] = motors[i].getEncoderPosition().into(Units.Radians);
            }
            holding = true;
        }

        for (int i = 0; i < 4; i++) {
            double error = holdTarget[i] - motors[i].getEncoderPosition().into(Units.Radians);
            double power = Math.abs(error) <= 0.05 ? 0 : Range.clip(error * 0.08, -0.3, 0.3);
            motors[i].setThrottle(power);
            currentPower[i] = power;
        }
    }

    public void stopNow() {
        for (int i = 0; i < 4; i++) {
            currentPower[i] = 0;
            motors[i].setThrottle(0.0);
        }
    }

    private static double shape(double input) {
        double magnitude = Math.abs(input);
        if (magnitude < 0.05) return 0;
        return Math.signum(input) * magnitude * magnitude;
    }

    @Override
    public void periodic() {}
}
