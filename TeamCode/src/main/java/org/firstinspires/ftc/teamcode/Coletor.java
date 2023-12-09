package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Coletor{

    private DcMotor motorCollector;
    private LinearOpMode opMode;

    private float collectorTrigger;

    public Coletor(LinearOpMode opMode){
        this.opMode = opMode;

        motorCollector = opMode.hardwareMap.get(DcMotor.class, "ColetorMotor");
    }

    public void periodic(){

        telemetry.addData("Coletor", " ");
        telemetry.update();




    }

    public void collect(double col){
        motorCollector.setPower(col);
        telemetry.addData("Coletor", motorCollector.getCurrentPosition());
        telemetry.update();

    }

    public void repelir(double rep){
        motorCollector.setPower(rep);
        telemetry.addData("Repelir pixel: ", motorCollector.getCurrentPosition());

    }
}
