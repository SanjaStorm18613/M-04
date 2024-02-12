/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Braco;
import org.firstinspires.ftc.teamcode.Subsystems.Coletor;
import org.firstinspires.ftc.teamcode.Subsystems.DriveMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Lancador;
import org.firstinspires.ftc.teamcode.Subsystems.SistemaLinear;


@TeleOp(name = "TeleopM04", group = "Linear OpMode")

public class TeleopM04 extends LinearOpMode {
    DriveMecanum driveMecanum;
    SistemaLinear sistemaLinear;
    Lancador lancador;
    Coletor coletor;
    Braco braco;

    private boolean bandejaBlock = false, bandejaBlockTotal = false, bandejaBlockTrava = false,
            motorPitch = false, inverterMotorElevador = false, inverterMotorBraco = false, inverterMotorPitch = false;

    @Override
    public void runOpMode() {

        driveMecanum  = new DriveMecanum (this);
        sistemaLinear = new SistemaLinear(this);
        lancador      = new Lancador     (this);
        coletor       = new Coletor      (this);
        braco         = new Braco        (this);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            // DroneLauncher
            lancador.lancarDrone(gamepad1.y);
            lancador.droneSetZero(gamepad1.dpad_up);

            //COLETOR
            coletor.collectorControl(gamepad1.left_trigger, -gamepad1.right_trigger);

            //BRACO
            braco.pitch((int)gamepad2.left_trigger * 100, (int)gamepad2.right_trigger * 100);

            if(gamepad2.back && !inverterMotorBraco && braco.getMotorBraco().getDirection() == DcMotorSimple.Direction.REVERSE) {
                braco.inverterMotorForward();
            }
            if(gamepad2.back && !inverterMotorBraco && braco.getMotorBraco().getDirection() == DcMotorSimple.Direction.FORWARD) {
                braco.inverterMotorReverse();
            }
            inverterMotorBraco = gamepad2.back;

            //SistemaLinear
            sistemaLinear.movimentarSistema(gamepad1.left_bumper, gamepad1.right_bumper);
            sistemaLinear.setMode(gamepad1.back);

            inverterMotorElevador = gamepad1.back;

            //BANDEJA TESTE
            if(gamepad2.a && !bandejaBlock) {
                braco.bandeja.travarBandeja(); //Destravar
            }
            if(coletor.getMotorCollector().getPower() > .4) {
                braco.bandeja.getServoTravaBandeja().setPosition(.6);

            } else if(coletor.getMotorCollector().getPower() <= .4) {
                braco.bandeja.getServoTravaBandeja().setPosition(.3);

            } else {
                braco.bandeja.getServoTravaBandeja().setPosition(.3);

            }
            braco.bandeja.pitchControl(gamepad1.a, gamepad1.x);
            braco.bandeja.travaAutonomo();

            bandejaBlock = gamepad2.a;

            //Tração
            driveMecanum.driveControl(Math.floor(gamepad1.left_stick_x * 10) / 10,
                                      Math.floor(gamepad1.left_stick_y * 10) / 10,
                                    Math.floor(gamepad1.right_stick_x * 10) / 10,
                                                    gamepad1.b);

            telemetry.addData("bl", driveMecanum.getBL().getCurrentPosition());
            telemetry.addData("motorBraco", braco.getMotorBraco().getCurrentPosition());
            telemetry.update();
        }
    }
}