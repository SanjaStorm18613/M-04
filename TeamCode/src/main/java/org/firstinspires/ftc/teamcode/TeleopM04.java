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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;


@TeleOp(name="TeleopM04", group="Linear OpMode")

public class TeleopM04 extends LinearOpMode {

    DriveMecanum driveMecanum;
    SistemaLinear sistemaLinear;
    LancaDrone lancaDrone;
    Coletor coletor;
    Bandeja bandeja;
    Braco braco;

    private boolean armBlockUp = false, armBlockDown = false, bandejaBlock = false, bandejaBlockTotal = false, bandejaBlockRight = false,
                    bandejaBlockLeft = false, rollBandejaLeftBlock = false, rollBandejaRightBlock = false, sistemaLinearBlockDown = false;

    public TeleopM04() {
        driveMecanum  = new DriveMecanum(this);
        sistemaLinear = new SistemaLinear(this);
        lancaDrone    = new LancaDrone(this);
        coletor       = new Coletor(this);
        bandeja       = new Bandeja(this);
        braco         = new Braco(this);
    }

    @Override
    public void runOpMode() {

        //public static Controller pilot, copilot;
        while(opModeIsActive()) {
            //DroneLauncher
            lancaDrone.lancarDrone(gamepad1.a);

            //Collector
            coletor.collect(Math.floor(gamepad1.right_trigger * 10) / 10);

            //Arm
            if (gamepad1.dpad_up && !armBlockUp) {
                braco.BracoUp();
            } else if (gamepad1.dpad_down && !armBlockDown) {
                braco.BracoDown();
            }
            armBlockUp = gamepad1.dpad_up;
            armBlockDown = gamepad1.dpad_down;


            //ao retrair o sistema, o roll volta para a posição inicial

            if (gamepad1.a) {
                sistemaLinear.retrairSistemaTotal();
                bandeja.rollBandeja(0);
            }

            //SistemaLinear
            sistemaLinear.esticarSistema(gamepad1.right_bumper);
            sistemaLinear.retrairSistema(gamepad1.left_bumper);


            if (gamepad1.b && !sistemaLinearBlockDown){
                sistemaLinear.retrairSistemaTotal();
            }
            sistemaLinearBlockDown = gamepad1.b;

            //Bandeja
            bandeja.pitchBandeja(braco.getTargetPos(), (Math.floor(gamepad2.right_trigger * 100) / 100));

            /** logica de girar e pontuar o pixel desejado -> ao pressionar o botão (a por exemplo), o sistema de entrega entende
            // que o pixel desejado para pontuar seja o da direita. com isso, o roll gira para a direita e destrava o pixel da direita,
            // lógica valida para o pixel da direita também (apertando o b, por exemplo):                                                          */


            if (gamepad2.a && !bandejaBlockRight){
                bandeja.rollBandeja(1);
                bandeja.destravarBandeja();

            } else if (gamepad2.b && !bandejaBlockLeft){
                bandeja.rollBandeja(-1);
                bandeja.destravarBandeja();

            }
            bandejaBlockRight = gamepad2.a;
            bandejaBlockLeft  = gamepad2.b;

            /** lógica de roll e destravar a partir de 2 botões diferentes, primeiro o piloto aperta o botão de roll, depois o de destravar  */

            if (gamepad2.left_bumper && !bandejaBlock){
                bandeja.destravarBandeja();
            } else if (gamepad2.right_bumper && !bandejaBlockTotal){
                bandeja.destravarBandejaTotal();
                bandeja.rollBandeja(1);
            }
            bandejaBlock      = gamepad2.left_bumper;
            bandejaBlockTotal = gamepad2.right_bumper;

            if (gamepad2.dpad_right && !rollBandejaRightBlock){
                bandeja.rollBandeja(1);
            } else if (gamepad2.dpad_left && !rollBandejaLeftBlock){
                bandeja.rollBandeja(-1);
            }
            rollBandejaRightBlock = gamepad2.dpad_right;
            rollBandejaLeftBlock  = gamepad2.dpad_left;

        }
    }
}