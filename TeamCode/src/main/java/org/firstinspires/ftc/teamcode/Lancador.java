package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Lancador {
    private Servo servoDrone;
    private LinearOpMode opMode;

    public Lancador(LinearOpMode opMode){
        this.opMode = opMode;

        servoDrone = opMode.hardwareMap.get(Servo.class, "Drone");
        servoDrone.setDirection(Servo.Direction.FORWARD);
    }

    public void periodic(){}
    public void lancarDrone(){
        servoDrone.setPosition(1);
    }

    public Servo getServoDrone(){
        return this.servoDrone;
    }
    public Servo setServoDrone(Servo servoDrone){
        this.servoDrone = servoDrone;
        return servoDrone;
    }
}