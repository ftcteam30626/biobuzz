package org.firstinspires.ftc.teamcode.teleopVariants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.utilities.OdometryTracker;

@TeleOp(name = "Odometry Example", group = "Utilities")
public class OdometryExample extends OpMode {

    private OdometryTracker odometry;
    private DcMotor leftOdo;
    private DcMotor rightOdo;
    private DcMotor strafeOdo;

    @Override
    public void init() {
        // Initialize the three odometry encoders
        leftOdo = hardwareMap.get(DcMotor.class, "leftOdo");      // Left parallel encoder
        rightOdo = hardwareMap.get(DcMotor.class, "rightOdo");    // Right parallel encoder
        strafeOdo = hardwareMap.get(DcMotor.class, "strafeOdo");  // Perpendicular (strafe) encoder

        // Create odometry tracker
        odometry = new OdometryTracker(leftOdo, rightOdo, strafeOdo);

        telemetry.addLine("Odometry tracker initialized!");
        telemetry.addLine("Move the robot around the field and watch position update");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update odometry every loop
        odometry.update();

        // Display robot position
        telemetry.addLine("=== ROBOT POSITION ===");
        telemetry.addData("X Position (in)", odometry.getX());
        telemetry.addData("Y Position (in)", odometry.getY());
        telemetry.addData("Heading (degrees)", odometry.getHeadingDegrees());
        telemetry.addLine();
        telemetry.addData("Position String", odometry.getPositionString());

        // Show encoder positions for debugging
        telemetry.addLine();
        telemetry.addLine("=== ENCODER VALUES ===");
        telemetry.addData("Left Encoder", leftOdo.getCurrentPosition());
        telemetry.addData("Right Encoder", rightOdo.getCurrentPosition());
        telemetry.addData("Strafe Encoder", strafeOdo.getCurrentPosition());

        // Reset position with A button
        if (gamepad1.a) {
            odometry.resetPosition();
            telemetry.addLine("Position Reset!");
        }

        telemetry.update();
    }
}
