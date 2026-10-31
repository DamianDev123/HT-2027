package org.firstinspires.ftc.teamcode.Solvers.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class ResetEncoders extends OpMode {

    public DcMotorEx turret;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "Turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Turret encoder reset!");
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}