package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
@TeleOp
@Config
public class testOp3 extends OpMode {
    ElapsedTime timer = new ElapsedTime();
    public static double power1 = 1;
    public static double power2 = 1;

    public static double power3 = -1;

    public static double power4 = 1;
    public static double power5 = 0.0;

    public static double power6 = 0.0;
    public MotorEx intakeMotor;
    public MotorEx transferMotor;
    public MotorEx shooterMotor;
    public MotorEx shooterAMotor;
    public ServoEx hood;
    public ServoEx flap;
    public ServoEx hang;
    public GamepadEx driver;
    @Override
    public void init() {
        intakeMotor = new MotorEx(hardwareMap, "Intake")
                .setCachingTolerance(0.01);
        transferMotor = new MotorEx(hardwareMap, "Transfer")
                .setCachingTolerance(0.01);
        shooterMotor = new MotorEx(hardwareMap, "Shooter")
                .setCachingTolerance(0.01);
        shooterAMotor = new MotorEx(hardwareMap, "shooterAssist")
                .setCachingTolerance(0.01);
        driver = new GamepadEx(gamepad1);
        timer.startTime();
        timer.reset();
    }

    @Override
    public void loop() {
        if(timer.milliseconds()>7000){
        transferMotor.set(power2);
       intakeMotor.set(power1);
        }else {

       shooterMotor.set(power3);
       shooterAMotor.set(power4);}

       // power2 = gamepad1.right_trigger>0.6? -1:0;

    }
}