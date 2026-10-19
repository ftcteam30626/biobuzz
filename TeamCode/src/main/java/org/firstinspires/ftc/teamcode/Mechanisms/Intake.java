package org.firstinspires.ftc.teamcode.Mechanisms;
import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {

    public final NextMotor intakeMotor = new NextMotor("intakeMotor");
    public void onInit() {
        stopIntake().schedule();
    }
    public void onStart() {
        runIntake().schedule();
    }
    public Command runIntake() {
        return infinite(() -> intakeMotor.setThrottle(1.0));
    }
    public Command stopIntake() {
        return instant(() -> intakeMotor.setThrottle(0.0));
    }
}