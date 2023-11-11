package org.firstinspires.ftc.teamcode;

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
        motorBraco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBraco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armTelemetry = opMode.telemetry;

    }


    public void periodic(double adjust) {

        targetPos = stages[stage] + adjust * Constants.Braco.adjust;

        motorBraco.setTargetPosition((int)(targetPos));
        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBraco.setPower(0.8);

        armTelemetry.addData("braço", targetPos);
        armTelemetry.update();
    }

    public double getTargetPos() {
        return targetPos;
    }

    public void BracoUp(){
            stage = min((stage + 1), 3);
    }
    public void BracoDown(){
            stage = max((stage - 1), 0);
    }

    public void testBraço(boolean a, boolean b, int c) {
        if(a) {
            BracoUp();
            armTelemetry.log();
        }
    }
}
