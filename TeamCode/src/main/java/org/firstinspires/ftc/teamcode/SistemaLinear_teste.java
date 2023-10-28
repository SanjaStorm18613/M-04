package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SistemaLinear_teste {

    private DcMotor motorBraco;
    private LinearOpMode opMode;

    public SistemaLinear_teste(LinearOpMode opMode){

        this.opMode = opMode;

        motorBraco = opMode.hardwareMap.get(DcMotor.class, "BracoMotor");
        motorBraco.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBraco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void periodic(){

        motorBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBraco.setTargetPosition(5000);
        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBraco.setPower(.8);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        public void wait(){
            opMode.waitForStart();
        }

        if(motorBraco.getCurrentPosition() < motorBraco.getTargetPosition()){
            telemetry.addData("encoder_motorbraco", motorBraco.getCurrentPosition() + " busy=")
        }

        motorBraco.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBraco.setTargetPosition(0);
        motorBraco.setPower(-0.8);


    }

}


