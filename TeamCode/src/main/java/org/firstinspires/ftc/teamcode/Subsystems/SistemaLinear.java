package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SistemaLinear {
    private DcMotor armMotor;

    private LinearOpMode opMode;

    //private TouchSensor limit;

    private int pos = 0, lastPos = 0;

    private boolean manterPot = false, manterBlock = false, inverterDirecao = true, travaInverter = false;

    public SistemaLinear(LinearOpMode opMode) {

        this.opMode = opMode;

        armMotor = opMode.hardwareMap.get(DcMotor.class, "MotorBraco");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Uso de uma limit para detectar o nível do braço
     */
    /*public void resetEnc(){
        if (!limit.isPressed()){
            armMotor.setPower(0.2);
        } else {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void retrairSistemaTotal(double power, int target){
        if (armMotor.getCurrentPosition() > 100){
            armMotor.setTargetPosition(target);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(.6);
        }
    }*/
    public void movimentarSistema(boolean Up, boolean Down) {

        /*pos += (Up ? 1 : 0) * 20;
        pos -= (Down ? 1 : 0) * 20;
        pos  = Math.max(pos, 0);
        armMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(1);*/

        if (inverterDirecao) {
            if (Up || Down) {
                lastPos = armMotor.getCurrentPosition();
                pos = lastPos + (Up ? 2000 : (Down ? -2000 : 0));
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

        opMode.telemetry.addData("power elevador", armMotor.getPower());
        //opMode.telemetry.update();

    }

    public void potenciaMotorElevador(boolean manter) {
        if (manter && !manterBlock) {
            manterPot = !manterPot;
        }
        manterBlock = manter;
    }

    public void inverterMotorForward() {
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void inverterMotorReverse() {
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
