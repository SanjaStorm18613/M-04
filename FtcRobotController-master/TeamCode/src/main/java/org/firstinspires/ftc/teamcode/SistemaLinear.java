package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SistemaLinear {
    private DcMotor armMotor;
    private LinearOpMode opMode;
    private boolean bumperUp, bumperDown, buttonDown;

    public SistemaLinear(LinearOpMode opMode){

        this.opMode = opMode;

        armMotor = opMode.hardwareMap.get(DcMotor.class, "AM");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        (opMode.gamepad2.left_bumper , gamepad2.right_bumper);
        setDescer(gamepad2.x);

    }

    public void periodic(){
        if(bumperDown){

            armMotor.setPower(-0.75);
        }
        else if(bumperUp){
            armMotor.setPower(0.75);
        }
        else{
            armMotor.setPower(0);
        }

        if(buttonDown){
            if (armEncoder > 30) {
                armMotor.setPower(-0.75);
            }
            armMotor.setPower(0);
        }

    }

    public void setEsticar(boolean gBumperUp, boolean gBumperDown){
        bumperUp = gBumperUp;
        bumperDown = gBumperDown;
    }
    public void setDescer(boolean gButtonDown){
        buttonDown = gButtonDown;
    }


}
