package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.Constants.Pitch.PITCH_LOW;
import static org.firstinspires.ftc.teamcode.Constants.Pitch.PITCH_UP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Base64;

public class SistemaLinear {
    private DcMotor armMotor;
    private LinearOpMode opMode;
    private Servo servoGancho, servoPitchBraco;
    private boolean bumperUp, bumperDown, buttonDown, buttonGanchoUp, bumperPitchBracoL;


    public SistemaLinear(LinearOpMode opMode){

        this.opMode = opMode;

        armMotor = opMode.hardwareMap.get(DcMotor.class, "MotorBraco");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoGancho = opMode.hardwareMap.get(Servo.class, "GanchoServo");
        servoGancho.setDirection(Servo.Direction.FORWARD);

        servoPitchBraco = opMode.hardwareMap.get(Servo.class, "ServoPitchBraco");
        servoPitchBraco.setDirection(Servo.Direction.FORWARD);

        bumperUp = opMode.gamepad2.left_bumper;
        bumperDown = opMode.gamepad2.right_bumper;
        buttonDown = opMode.gamepad2.x;
        buttonGanchoUp = opMode.gamepad1.y;
        bumperPitchBracoL = opMode.gamepad1.right_bumper;

    }

    public void periodic(){
        if(bumperDown && armEncoder > 30) {
            armMotor.setPower(-0.75);
        }
        else if(bumperUp && armEncoder < 150) {
            armMotor.setPower(0.75);
        }
        else {
            armMotor.setPower(0);
        }

        if(buttonGanchoUp){
            if(armEncoder > 30) {
                armMotor.setPower(-0.75);
            }
            else{
                armMotor.setPower(0);
            }
            servoGancho.setPosition(.6);
        }

        if(buttonDown){
            if (armEncoder > 30) {
                armMotor.setPower(-0.75);
            }
            else {
                armMotor.setPower(0);
            }
        }

        if(bumperPitchBracoL){
            servoPitchBraco.setPosition(1);
        }
    }

}
