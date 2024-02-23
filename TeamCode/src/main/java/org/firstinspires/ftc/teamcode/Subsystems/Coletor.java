package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleopM04;

public class Coletor{
    private DcMotor motorCollector;
    private LinearOpMode opMode;

    public Coletor(LinearOpMode opMode){
        this.opMode = opMode;
        motorCollector = opMode.hardwareMap.get(DcMotor.class, "ColetorMotor");
        motorCollector.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void collectorControl(double powerC, double powerR){
        motorCollector.setPower(powerC + powerR);
    }
    public DcMotor getMotorCollector(){
        return motorCollector;
    }
}