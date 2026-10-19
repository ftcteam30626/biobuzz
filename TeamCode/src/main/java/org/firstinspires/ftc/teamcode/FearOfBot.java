package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class FearOfBot implements NextRobot {
    Drivetrain drivetrain = new Drivetrain();
    Intake intake = new Intake();
    Outtake outtake = new Outtake();
    Transfer transfer = new Transfer();

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drivetrain, intake, transfer, outtake);
    }
}