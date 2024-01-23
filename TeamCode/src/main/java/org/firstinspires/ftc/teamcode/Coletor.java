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
        motorCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCollector.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void collect(double power){
        this.power = power;
        motorCollector.setPower(power);
    }
    public void repel(double power){
        this.power = power;
        motorCollector.setPower(power);
    }

    public void repelAuto(){
        motorCollector.setPower(1);
    }
    public DcMotor getMotorCollector(){
        return motorCollector;
    }
}