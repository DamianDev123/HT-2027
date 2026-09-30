package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@TeleOp
@Config
public class testOp extends OpMode {
    public ServoEx turretServo1;
    public ServoEx turretServo2;
    public MotorEx motor;
    public GamepadEx driver;

    // --- TUNING CONTROLS ---
    public static double forward = 0.63; // Your 0 degree position
    public static double left = 0.1;    // Your 90 degree position
    public static double backward = 0.56; // Your 0 degree position
    public static double right = 0.04;    // Your 90 degree position

    // Set this in Dashboard to any degree (e.g., 0, 90, 180, -45)
    public static double targetDegrees = 0.0;

    @Override
    public void init() {
        turretServo1 = new ServoEx(hardwareMap, "turretServo1")
                .setCachingTolerance(0.01);
        turretServo2 = new ServoEx(hardwareMap, "turretServo2")
                .setCachingTolerance(0.01);
        motor = new MotorEx(hardwareMap, "Shooter");
        driver = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        // 1. Find the "Range" (how many servo units represent 90 degrees)s
        double servoPosition = toServo(targetDegrees,true);
        turretServo1.set(servoPosition);
        turretServo2.set(servoPosition);

        // Telemetry for debugging
        telemetry.addData("Target Angle", targetDegrees);
        telemetry.addData("Calculated PWM", servoPosition);
        telemetry.update();
    }
    public static double toServo(double targetDegrees, boolean backwards){
        if (backwards) {
            if (targetDegrees > 0) {
                targetDegrees -= 360.0;
            }
            // Since forward is 0, backward is 180 and right is 270.
            // This maps the range from 180 to 270 degrees.
            return backward + ((right - backward) * ((targetDegrees + 180.0) / 90.0));
        }
        return forward + ((left - forward) * (targetDegrees / 90.0));

    }
    public static double toServo(double targetDegrees){
        return forward + ((left - forward) * (targetDegrees / 90.0));
    }
    public static double toDegree(double servo, boolean backwards) {
        if(backwards){
            double d = ((right - backward));
            double degrees = ((((servo-backward)/d))* 90.0)-180;
            if (degrees > 0) {
                degrees -= 360.0;
            }
            return degrees;
        }
        double d = ((left - forward));
        return ((((servo-forward)/d))* 90.0);
    }
    public static double toDegree(double servo){
        double d = ((left - forward));
        return ((((servo-forward)/d))* 90.0);
    }

}