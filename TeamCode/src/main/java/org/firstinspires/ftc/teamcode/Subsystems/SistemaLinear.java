package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SistemaLinear {
    private DcMotor armMotor;

    private LinearOpMode opMode;

    //private TouchSensor limit;

    private int pos = 0, lastPos = 0;

    private boolean inverterDirecao = true, travaInverter = false;

    public SistemaLinear(LinearOpMode opMode) {

        this.opMode = opMode;

        armMotor = opMode.hardwareMap.get(DcMotor.class, "MotorBraco");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    /*public void resetEnc(){
        if (!limit.isPressed()){
            armMotor.setPower(0.2);
        } else {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }*/
    public void movimentarSistema(boolean Up, boolean Down) {
        if (inverterDirecao) {
            if (Up || Down) {
                lastPos = armMotor.getCurrentPosition();
                pos = lastPos + (Up ? 3000 : (Down ? -3000 : 0));
            } else {
                pos = lastPos;
            }

            pos = Math.max(pos, 0);

            armMotor.setTargetPosition(pos);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
        } else {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(Up ? 1 : (Down ? -1 : 0));
        }
    }
    public void setMode(boolean inverterModo) {
        if (inverterModo && !travaInverter) {
            if (!inverterDirecao) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                pos = 0;
            }
            inverterDirecao = !inverterDirecao;
        }

        travaInverter = inverterModo;
    }
    public DcMotor getArmMotor() {
        return armMotor;
    }
}
