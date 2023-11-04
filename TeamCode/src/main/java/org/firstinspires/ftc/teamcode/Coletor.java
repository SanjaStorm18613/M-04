package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Coletor{

    private DcMotor motorCollector;
    private LinearOpMode opMode;
    private float collectorTrigger;

    public Coletor(LinearOpMode opMode){
        this.opMode = opMode;

        motorCollector = opMode.hardwareMap.get(DcMotor.class, "ColetorMotor");
    }

    public void periodic(){

    }

    public void collect(double col){
        motorCollector.setPower(col);
    }
}
