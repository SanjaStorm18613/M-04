package org.firstinspires.ftc.teamcode;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Braco extends Constants.Braco {

    private DcMotor motorBraco;
    private Servo servoPitch;
    private LinearOpMode opMode;
    private boolean bracoButtonUp, bracoButtonDown;
    private double triggerPitchUp, triggerPitchDown;

    private final double[] stages = { Constants.Braco.stage0, Constants.Braco.stage1,
                                     Constants.Braco.stage2, Constants.Braco.stage3 };

    private int stage = 0;
    private double targetPos, adjust = 0;

    public Braco(LinearOpMode opMode){
        this.opMode = opMode;

        motorBraco = opMode.hardwareMap.get(DcMotor.class, "BracoMotor");
        motorBraco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBraco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoPitch = opMode.hardwareMap.get(Servo.class, "ServoPitch");
        servoPitch.setDirection(Servo.Direction.FORWARD);

    }

    public void periodic() {

        if(bracoButtonUp) {
            stage = min((stage + 1),3);
        } else if (bracoButtonDown) {
            stage = max((stage - 1) ,0);
        }

        adjust = opMode.gamepad1.right_trigger;

        targetPos = stages[stage] + adjust * Constants.Braco.adjust;

        motorBraco.setTargetPosition((int)(targetPos));
        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBraco.setPower(0.8);

    }

    /*public void Esticar(double gTargetPosUp){
        gTargetPosUp = targetPos;
        targetPos = stage++;
    }

    public void Retrair(double gTargetPosDown){
        gTargetPosDown = targetPos;
        targetPos = stage--;
    }*/

}
