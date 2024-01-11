package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Coletor{
    private DcMotor motorCollector;
    private LinearOpMode opMode;

    private double power;

    public Coletor(LinearOpMode opMode){
        this.opMode = opMode;
        motorCollector = opMode.hardwareMap.get(DcMotor.class, "ColetorMotor");
        motorCollector.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void periodic(){}

    public void collect(double power){
        motorCollector.setPower(power);
        //opMode.telemetry.addData("core hex", motorCollector.getCurrentPosition());
        //opMode.telemetry.update();
    }
    public void repel(double power){
        motorCollector.setPower(power);
        //opMode.telemetry.addData("core hex", motorCollector.getCurrentPosition());
        //opMode.telemetry.update();
    }
}