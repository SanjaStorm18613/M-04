package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lancador {
    private Servo servoDrone;
    private LinearOpMode opMode;

    public Lancador(LinearOpMode opMode){
        this.opMode = opMode;

        servoDrone = opMode.hardwareMap.get(Servo.class, "Drone");
        servoDrone.setDirection(Servo.Direction.FORWARD);

    }
    public void droneSetZero(boolean lancar){
        if(lancar) servoDrone.setPosition(.4);
    }
    public void launchDrone(boolean adjust){
        if(adjust) servoDrone.setPosition(.2);
    }
}