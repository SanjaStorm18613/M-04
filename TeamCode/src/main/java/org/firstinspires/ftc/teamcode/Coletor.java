package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Coletor {

    private DcMotor motorCollector;
    private LinearOpMode opMode;
    private float collectorTrigger;

    public Coletor(LinearOpMode opMode){

        this.opMode = opMode;

        motorCollector = opMode.hardwareMap.get(DcMotor.class, "ColetorMotor");

        collectorTrigger = opMode.gamepad1.right_trigger;
    }

    public void periodic(){
        if(collectorTrigger > .1){
            motorCollector.setPower(1);
        }
        else{
            motorCollector.setPower(0);
        }
    }
}
