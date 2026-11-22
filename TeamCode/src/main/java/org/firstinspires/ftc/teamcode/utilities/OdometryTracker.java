package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometryTracker {

    private final DcMotor leftOdometer;
    private final DcMotor rightOdometer;
    private final DcMotor strafeOdometer;

    private long lastLeftPos = 0;
    private long lastRightPos = 0;
    private long lastStrafePos = 0;

    private double worldX = 0;  // Position in inches (X-axis)
    private double worldY = 0;  // Position in inches (Y-axis)
    private double heading = 0; // Heading in radians

    private static final double TICKS_PER_INCH = 50; // Adjust based on your encoder resolution
    private static final double ODOMETER_DISTANCE = 12; // Distance between left/right odometers in inches

    public OdometryTracker(DcMotor left, DcMotor right, DcMotor strafe) {
        this.leftOdometer = left;
        this.rightOdometer = right;
        this.strafeOdometer = strafe;
    }

    public void update() {
        long leftPos = leftOdometer.getCurrentPosition();
        long rightPos = rightOdometer.getCurrentPosition();
        long strafePos = strafeOdometer.getCurrentPosition();

        double leftDelta = (leftPos - lastLeftPos) / TICKS_PER_INCH;
        double rightDelta = (rightPos - lastRightPos) / TICKS_PER_INCH;
        double strafeDelta = (strafePos - lastStrafePos) / TICKS_PER_INCH;

        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        lastStrafePos = strafePos;

        // Calculate change in heading (rotation)
        double deltaHeading = (rightDelta - leftDelta) / ODOMETER_DISTANCE;
        heading += deltaHeading;

        // Calculate average movement
        double avgForwardDelta = (leftDelta + rightDelta) / 2.0;

        // Update position based on current heading
        worldX += avgForwardDelta * Math.cos(heading) - strafeDelta * Math.sin(heading);
        worldY += avgForwardDelta * Math.sin(heading) + strafeDelta * Math.cos(heading);
    }

    public double getX() {
        return worldX;
    }

    public double getY() {
        return worldY;
    }

    public double getHeading() {
        return heading;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }

    public void resetPosition() {
        worldX = 0;
        worldY = 0;
        heading = 0;
        lastLeftPos = leftOdometer.getCurrentPosition();
        lastRightPos = rightOdometer.getCurrentPosition();
        lastStrafePos = strafeOdometer.getCurrentPosition();
    }

    public void setPosition(double x, double y, double heading) {
        this.worldX = x;
        this.worldY = y;
        this.heading = heading;
    }

    public String getPositionString() {
        return String.format("X: %.2f, Y: %.2f, Heading: %.2f°", worldX, worldY, getHeadingDegrees());
    }
}
