package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Braco {

    private DcMotor motorBraco;
    private Servo servoPitch;
    private LinearOpMode opMode;
    private final double[] stages = { Constants.Braco.stage0, Constants.Braco.stage1,
                                      Constants.Braco.stage2, Constants.Braco.stage3 };

    Telemetry armTelemetry;

    private int stage = 0;
    private double targetPos, adjust = 0;

    private double[] resp = {0, 1, 2, 3, 4, 5, 6, 6, 7, 7};

    public Braco(LinearOpMode opMode){

        this.opMode = opMode;

        motorBraco = opMode.hardwareMap.get(DcMotor.class, "BracoMotor");

        motorBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBraco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBraco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armTelemetry = opMode.telemetry;

    }

    public void periodic(double adjust) {

        targetPos = stages[stage] + adjust * Constants.Braco.adjust;

        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBraco.setTargetPosition((int)(targetPos));
        motorBraco.setPower(0.8);

        telemetry.addData("Braco", targetPos);
        telemetry.update();
    }

    public double getTargetPos() {
        return targetPos;
    }

    /*public void BracoUp(){
            stage = min((stage + 1), 3);
            motorBraco.setPower(stage);
    }
    public void BracoDown(double power){
            stage = max((stage - 1), 0);
            motorBraco.setPower(power);
    }*/

    public void PitchBraco(double power, int target){

        if(motorBraco.getCurrentPosition() >= Constants.Braco.stage0 && motorBraco.getCurrentPosition() <= Constants.Braco.stage1) {

            motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBraco.setTargetPosition(target);
            motorBraco.setPower(power);

            telemetry.addData("MotorBraço", motorBraco.getCurrentPosition());
            telemetry.update();

        }

        else if(motorBraco.getCurrentPosition() < Constants.Braco.stage0 || motorBraco.getCurrentPosition() > Constants.Braco.stage1){

            motorBraco.setPower(0);
            telemetry.addData("MotorBraço", motorBraco.getCurrentPosition());
            telemetry.update();

        }
    }

}
