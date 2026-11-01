package org.firstinspires.ftc.teamcode.utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Utility(name="Servo Position Helper")
public class ServoPositionHelper extends LinearOpMode {
    // Declare OpMode member.
    private Servo servo = null;

    /*
     * Create a variable which we will modify with our code. Eventually we will instruct
     * the servo to run to the position captured by this variable.
     */
    private double servoPosition = 0.5;

    // Create a variable for size of each "step" that we will increment or decrement our servo position by.
    private double positionAdjustment = 0.05;

    // This variable captures how much we need to increment or decrement the step size by
    private final double STEP_ADJUSTMENT = 0.01;

    /*
     * This variable is the maximum position we want to send to the servo.
     * Some servos do not operate well went sent a signal too large, or too small.
     * Most Hitec Linear servos for example only respond to signals within a 1050-1950µsec range.
     * Converted to 0-1, that means we should not send a Hitec Linear Servo less than 0.25, or more than 0.75.
     */
    private final double MIN_POSITION = 0;
    private final double MAX_POSITION = 1;

    // These booleans are used in the "rising edge detection"
    private boolean previousGamepadY = false;
    private boolean previousGamePadA = false;
    private boolean previousGamePadUp = false;
    private boolean previousGamePadDown = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Completed");
        servo = hardwareMap.get(Servo.class, "transferServo");

        servoPosition = 0.5;
        servo.setPosition(servoPosition);
        telemetry.addData("Servo Set Position: ",servo.getPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean currentGamepadY = gamepad1.y;
            boolean currentGamepadA = gamepad1.a;
            boolean currentGamepadUp = gamepad1.dpad_up;
            boolean currentGamepadDown = gamepad1.dpad_down;

            // Check to see if the user is clicking the Y(△) button on the gamepad.
            if (currentGamepadY && !previousGamepadY){
                // += is an operator that lets us add the step variable without overwriting the servoPosition variable.
                servoPosition += positionAdjustment;
            } else if (currentGamepadA && !previousGamePadA){
                // We use an else if statement here so that we only check if A(x) is pressed after we know
                // that the Y(△) button is not pressed.
                servoPosition -= positionAdjustment;
            }

            // Here we modify the step size if the user clicks D-pad up or D-pad down.
            if (currentGamepadUp && !previousGamePadUp){
                positionAdjustment += STEP_ADJUSTMENT;
            } else if (currentGamepadDown && !previousGamePadDown){
                positionAdjustment -= STEP_ADJUSTMENT;
            }

            // Check to see if we're setting the servoPosition to less than the min, or more than the max.
            if (servoPosition > MAX_POSITION){
                servoPosition = MAX_POSITION;
            } else if (servoPosition < MIN_POSITION){
                servoPosition = MIN_POSITION;
            }

            /*
             * Finally, set the servo to the servoPosition variable. We do this only once per loop
             * so that we can be sure not to write conflicting positions to the servo.
             */
            servo.setPosition(servoPosition);

            // Because our logic has finished, we set our "previousGamepad" booleans to the current ones.
            previousGamepadY = currentGamepadY;
            previousGamePadA = currentGamepadA;
            previousGamePadUp = currentGamepadUp;
            previousGamePadDown = currentGamepadDown;

            // Show the servo position
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Servo Step Size", positionAdjustment);
            telemetry.update();
        }
    }
}