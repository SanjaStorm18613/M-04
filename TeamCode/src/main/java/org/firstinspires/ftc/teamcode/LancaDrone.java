package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import android.accounts.AbstractAccountAuthenticator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LancaDrone {

    private Servo servoDrone, servoPitchDrone;
    private LinearOpMode opMode;
    private Telemetry droneTelemetry;

    public LancaDrone(LinearOpMode opMode){
        this.opMode = opMode;

        servoDrone = opMode.hardwareMap.get(Servo.class, "Drone");
        servoDrone.setDirection(Servo.Direction.FORWARD);

        droneTelemetry = opMode.telemetry;

    }

    public void periodic(){

        lancarDrone();
        droneTelemetry.addData("Drone", servoDrone.getPosition());
        droneTelemetry.update();

    }

    public void lancarDrone(){
        servoDrone.setPosition(.7);
    }

}