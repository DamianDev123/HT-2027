package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumTeleOp", group = "Drive")
@Config
public class MecanumTeleOp extends OpMode {

    private DcMotor frontLeft;
    ElapsedTime elapsedTime = new ElapsedTime();
    public static double flapPos = 0.0;
    public static double flapPosClose = 0.34;
    public static double flapPosOpen = 0.63;


    private DcMotor frontRight;
    private DcMotor backLeft;
    private Servo flap;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotor transfer;
    private DcMotor shooter;

    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        flap = hardwareMap.get(Servo.class, "Flap");

        // Reverse left side motors
        // Change these if your robot drives incorrectly
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake when sticks are released
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elapsedTime.startTime();
        elapsedTime.reset();
    }

    @Override
    public void loop() {

        // Gamepad inputs
        double y = -gamepad1.left_stick_y;   // Forward/back
        double x = gamepad1.left_stick_x;    // Strafe
        double rx = gamepad1.right_stick_x;  // Turn

        // Mecanum math
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Slow mode
        double speedMultiplier = gamepad1.right_bumper ? 0.35 : 1.0;

        // Apply power
        frontLeft.setPower(frontLeftPower * speedMultiplier);
        backLeft.setPower(backLeftPower * speedMultiplier);
        frontRight.setPower(frontRightPower * speedMultiplier);
        backRight.setPower(backRightPower * speedMultiplier);
        if(gamepad1.a)
            flap.setPosition(flapPosClose);
        if(gamepad1.b)
            flap.setPosition(flapPosOpen);

        shooter.setPower(1);

        if(true) {
            transfer.setPower(-1);
            intake.setPower(-1);
        }

        // Telemetry
        telemetry.addData("FL", frontLeftPower);
        telemetry.addData("FR", frontRightPower);
        telemetry.addData("BL", backLeftPower);
        telemetry.addData("BR", backRightPower);
        telemetry.update();
    }
}