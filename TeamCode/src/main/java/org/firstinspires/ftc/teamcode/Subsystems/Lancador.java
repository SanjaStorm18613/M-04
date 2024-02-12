package org.firstinspires.ftc.teamcode.Subsystems;

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
    public void lancarDrone(boolean lancar){
        if(lancar) servoDrone.setPosition(1);
    }
    public void droneSetZero(boolean dpad_up){
        if(dpad_up) servoDrone.setPosition(.2);
    }
}