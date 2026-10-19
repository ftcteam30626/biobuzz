package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Transfer implements Mechanism {

    public final NextMotor transferMotor = new NextMotor("transferMotor");
    public void onInit() {
        stopIntake().schedule();
    }
    public void onStart() {
        runIntake().schedule();
    }
    public Command runIntake() {
        return infinite(() -> transferMotor.setThrottle(1.0));
    }
    public Command stopIntake() {
        return instant(() -> transferMotor.setThrottle(0.0));
    }
}