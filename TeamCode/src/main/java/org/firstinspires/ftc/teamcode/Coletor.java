package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Coletor{

    private DcMotor motorCollector;
    private LinearOpMode opMode;

    Telemetry collectorTelemetry;
    private float collectorTrigger;

    public Coletor(LinearOpMode opMode){
        this.opMode = opMode;

        motorCollector = opMode.hardwareMap.get(DcMotor.class, "ColetorMotor");

        collectorTelemetry = opMode.telemetry;
        collectorTelemetry.addData("Coletor", "collect");
        collectorTelemetry.update();
    }

    public void periodic(){

    }

    public void collect(double col){
        motorCollector.setPower(col);
    }
}
