package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Utility(name = "Simple Drive with Gate")
public class SimpleDriveWithGate extends OpMode {

    private DcMotorEx leftFront, rightFront, rightBack, leftBack;
    private DcMotorEx[] motors;
    private Servo gate;

    private double[] currentPower = new double[4];
    private int[] holdTarget = new int[4];
    private boolean holding = false;
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean lastA = false, lastB = false;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        motors = new DcMotorEx[]{leftFront, rightFront, rightBack, leftBack};
        gate = hardwareMap.get(Servo.class, "gate");

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        gate.setPosition(0.8);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0.1) dt = 0.02;

        // A = open gate, B = close gate
        if (gamepad1.a && !lastA) gate.setPosition(0.1);
        if (gamepad1.b && !lastB) gate.setPosition(0.8);
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        // Drive
        double drive = shape(-gamepad1.left_stick_y);
        double strafe = shape(gamepad1.left_stick_x);
        double turn = gamepad1.right_stick_x > 0.05 || gamepad1.right_stick_x < -0.05 ? gamepad1.right_stick_x : 0;

        // If sticks are idle, hold position instead of coasting
        if (drive == 0 && strafe == 0 && turn == 0) {
            holdPosition();
        } else {
            holding = false;
            // Drive
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
                motors[i].setPower(currentPower[i]);
            }
        }

        telemetry.addData("Gate", "%.1f", gate.getPosition());
        telemetry.addData("Mode", holding ? "HOLDING" : "DRIVING");
        telemetry.addData("LF / RF", "%.2f / %.2f", currentPower[0], currentPower[1]);
        telemetry.addData("LB / RB", "%.2f / %.2f", currentPower[3], currentPower[2]);
        telemetry.update();
    }

    private void holdPosition() {
        // Lock in position the first time sticks go idle
        if (!holding) {
            for (int i = 0; i < 4; i++) {
                holdTarget[i] = motors[i].getCurrentPosition();
            }
            holding = true;
        }

        // Push each wheel back to locked spot (P controller)
        for (int i = 0; i < 4; i++) {
            int error = holdTarget[i] - motors[i].getCurrentPosition();
            double power = Math.abs(error) <= 3 ? 0 : Range.clip(error * 0.008, -0.3, 0.3);
            motors[i].setPower(power);
            currentPower[i] = power;
        }
    }

    @Override
    public void stop() {
        gate.setPosition(0.8);
        for (DcMotorEx motor : motors) motor.setPower(0);
    }

    private static double shape(double input) {
        double magnitude = Math.abs(input);
        if (magnitude < 0.05) return 0;
        return Math.signum(input) * magnitude * magnitude;
    }
}
