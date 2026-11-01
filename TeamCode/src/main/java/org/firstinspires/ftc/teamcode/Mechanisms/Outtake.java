package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Outtake implements Mechanism {

    public final NextMotor shooterMotor1 = new NextMotor("outtakeMotor1");
    public final NextMotor shooterMotor2 = new NextMotor("outtakeMotor2");
    public void onInit() {
        stopShooter1().schedule();
        stopShooter2().schedule();
    }
    public void onStart() {
        runShooter1().schedule();
        runShooters2().schedule();
    }
    public Command runShooter1() {
        return infinite(() -> shooterMotor1.setThrottle(1.0));
    }
    public Command runShooters2() {
        return infinite(() -> shooterMotor2.setThrottle(1.0));
    }
    public Command stopShooter1() {
        return instant(() -> shooterMotor2.setThrottle(0.0));
    }
    public Command stopShooter2() {
        return instant(() -> shooterMotor1.setThrottle(0.0));
    }
}
