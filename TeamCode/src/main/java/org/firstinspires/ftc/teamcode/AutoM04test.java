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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="AutoM04test", group="Robot")
//@Disabled

public class AutoM04test extends LinearOpMode {
    DriveMecanum driveMecanum;

    SistemaLinear sistemaLinear;

    Coletor coletor;
    int step;

    public AutoM04test(){

        telemetry.addData("Inicializando Autonomo", "Chassi pronto!");
        driveMecanum = new DriveMecanum(this, sistemaLinear);
        sistemaLinear = new SistemaLinear(this);
        coletor = new Coletor(this);
        step = 0;

        driveMecanum.resetEnc();
    }

    @Override
    public void runOpMode() {
        while (opModeIsActive()) {

            idle();

            telemetry.addData("DriveMecanumRun", driveMecanum.getBL().getCurrentPosition());
            telemetry.update();

            driveMecanum.moveForwardAuto(.7, 3801);

            if (driveMecanum.getBL().getCurrentPosition() >= 3800 && step == 0){ resetEnc_step(); }

            if (step == 1){ driveMecanum.turn(.7, 2871); }

            if (driveMecanum.getBL().getCurrentPosition() > ConstantsAuto.Drive.degree90 && step == 1){ resetEnc_step(); }

            if (step == 2){ driveMecanum.moveForwardAuto(.8, 11001); }

            if (driveMecanum.getBL().getCurrentPosition() > 11000 && step == 2) { resetEnc_step(); }

            if (step == 3){ driveMecanum.right(.7, 3301); }

            if (driveMecanum.getBL().getCurrentPosition() < -3300 && step == 3){ resetEnc_step(); }

            if (step == 4){ driveMecanum.moveForwardAuto(-0.7, -11001); }

            if (driveMecanum.getBL().getCurrentPosition() < -11000 && step == 4){ resetEnc_step(); }

            if (step == 5){ driveMecanum.turn(-0.7, -2871); }

            if (driveMecanum.getBL().getCurrentPosition() < -ConstantsAuto.Drive.degree90 && step == 5){ resetEnc_step(); }

            if (step == 6){ driveMecanum.moveForwardAuto(.8, 6001); }

            if (driveMecanum.getBL().getCurrentPosition() > 6000 && step == 6){ resetEnc_step(); }

            if (step == 7){ driveMecanum.turn(.8, 5742); }

            if (driveMecanum.getBL().getCurrentPosition() > 5741 && step == 7){ resetEnc_step(); }

            if (step == 8){ driveMecanum.moveForwardAuto(.5, -7000); }

            //coletor.repelirAuto(.5);


        }
    }

    public void resetEnc_step(){
        driveMecanum.resetEnc();
        step++;
    }
}
