package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.accounts.AbstractAccountAuthenticator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LancaDrone {

    private Servo servoDrone, servoPitchDrone;
    private LinearOpMode opMode;

    public LancaDrone(LinearOpMode opMode){
        this.opMode = opMode;

        servoDrone = opMode.hardwareMap.get(Servo.class, "Drone");
        servoDrone.setDirection(Servo.Direction.FORWARD);


    }

    public void periodic(){

        lancarDrone();
        telemetry.addData("ServoDrone", servoDrone.getPosition());
        telemetry.update();

    }

    public void lancarDrone(){
        servoDrone.setPosition(.7);
    }

}