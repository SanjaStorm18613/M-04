package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Braco {
    private DcMotor motorBraco;
    private LinearOpMode opMode;
    public int pos, encoder;
    private ElapsedTime timer;
    public Bandeja bandeja;

    public Braco(LinearOpMode opMode){

        this.opMode = opMode;

        motorBraco = opMode.hardwareMap.get(DcMotor.class, "BracoMotor");
        motorBraco.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBraco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBraco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bandeja = new Bandeja(this.opMode);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.startTime();

        encoder = motorBraco.getCurrentPosition();

    }
    public void resetEncoder(){
        motorBraco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public DcMotor getMotorBraco(){
        return this.motorBraco;
    }
    public void armControl(int up, int down){
        pos += up/100. * 20;
        pos -= down/100. * 15;
        pos  = Math.max(pos, 0);

        motorBraco.setTargetPosition(pos);
        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBraco.setPower(1);
    }

    public void pitchAuto(double power, int target){
        motorBraco.setTargetPosition(target);
        motorBraco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBraco.setPower(power);
    }

    public void inverterMotorForward(){
        motorBraco.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void inverterMotorReverse(){
        motorBraco.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
