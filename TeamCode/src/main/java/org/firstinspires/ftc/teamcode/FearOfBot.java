package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

/*
 * THE ROBOT
 *
 * This class holds every mechanism on the robot. NextFTC finds it automatically
 * (because it implements NextRobot) and hands it to every NextFTC OpMode.
 * Mechanisms listed in getMechanisms() get their periodic() called every loop.
 *
 * To add a new mechanism: make it in the Mechanisms folder, add a field here,
 * and add it to getMechanisms().
 */
public class FearOfBot implements NextRobot {
    // One of each mechanism. "public final" = OpModes can use them, but can't swap them out.
    public final Drivetrain drivetrain = new Drivetrain();
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    public final Transfer transfer = new Transfer();

    // Tells NextFTC which mechanisms to run periodic() on every loop.
    @Override
    public Set<Mechanism> getMechanisms() {
        // Set.of(...) would be shorter, but it crashes on the Control Hub's older Android version.
        return new HashSet<>(Arrays.asList(drivetrain, shooter, intake, transfer));
    }
}
