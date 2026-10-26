package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

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
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();

    @Override
    public Set<Mechanism> getMechanisms() {
        // Set.of(...) would be shorter, but it crashes on the Control Hub's older Android version.
        return new HashSet<>(Arrays.asList(shooter, intake));
    }
}
