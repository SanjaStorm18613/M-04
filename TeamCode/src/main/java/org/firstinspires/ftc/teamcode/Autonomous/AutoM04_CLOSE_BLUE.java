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

package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Braco;
import org.firstinspires.ftc.teamcode.Subsystems.Coletor;
import org.firstinspires.ftc.teamcode.Subsystems.DriveMecanum;
import org.firstinspires.ftc.teamcode.Subsystems.Lancador;
import org.firstinspires.ftc.teamcode.Subsystems.SistemaLinear;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

@Autonomous(name= "AutoM04_CLOSE_BLUE", group = "Robot")
//@Disabled
public class AutoM04_CLOSE_BLUE extends LinearOpMode {

    DriveMecanum driveMecanum;
    SistemaLinear sistemaLinear;
    Coletor coletor;
    Braco braco;
    ElapsedTime timer;
    ElementLoc loc;
    private AprilTagProcessor aprilTag;
    private Pipeline_Azul pipelineAzul;
    private VisionPortal visionPortal;
    int step = 0;


    @Override
    public void runOpMode(){
        telemetry.addData("Inicializando auto", " ");

        driveMecanum = new DriveMecanum(this);

        sistemaLinear = new SistemaLinear(this);

        coletor = new Coletor(this);

        braco = new Braco(this);

        timer = new ElapsedTime();

        timer.reset();

        initCamera();

        while(!isStarted() && !isStopRequested()){
            loc = pipelineAzul.getLocation();
            telemetry.addData("camera", loc);
            telemetry.update();
        }
        while (opModeIsActive()) {

            visionPortal.setProcessorEnabled(pipelineAzul,false);

            switch(loc){

                case CENTER:
                    if(step == 0) driveMecanum.moveForwardAuto(-0.7, 1300);

                    if(driveMecanum.getBL().getCurrentPosition() >= 1290 && step == 0) {
                        resetEnc_step();
                    }

                    if(step == 1) {
                        braco.bandeja.setAutoZero();
                        driveMecanum.setPowerZero();
                        timer.reset();
                        timer.startTime();
                    }

                    if(braco.bandeja.getTravaAuto().getPosition() >= .4 && step == 1) {
                        resetEnc_step();
                    }

                    if(step == 2 && timer.milliseconds() > 400){
                        driveMecanum.moveForwardAuto(.7, -600);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() <= -590 && step == 2){
                        resetEnc_step();
                    }
                    if(step == 3){
                        driveMecanum.right(.5, -2500);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() > 2490 && step == 3){
                        resetEnc_step();
                    }
                    if(step == 4) {
                        timer = new ElapsedTime();
                        timer.reset();
                        timer.startTime();
                        coletor.collectorControl(0, -0.6);
                        driveMecanum.setPowerZero();
                        resetEnc_step();
                    }

                    if(step == 5 && timer.seconds() > 5){
                        coletor.collectorControl(0, 0);
                        resetEnc_step();
                    }
                    break;
                case LEFT:
                    if(step == 0) driveMecanum.moveForwardAuto(-0.7, 1250);

                    if(driveMecanum.getBL().getCurrentPosition() >= 1240 && step == 0) {
                        resetEnc_step();
                    }
                    if(step == 1) driveMecanum.turn(.6, 900);

                    if(driveMecanum.getBL().getCurrentPosition() >= 890 && step == 1){
                        resetEnc_step();
                    }

                    if(step == 2){
                        braco.bandeja.setAutoZero();
                        driveMecanum.setPowerZero();
                        timer.reset();
                        timer.startTime();
                    }
                    if(braco.bandeja.getTravaAuto().getPosition() >= .4 && step == 2) {
                        resetEnc_step();
                    }
                    if(timer.milliseconds() > 400 && step == 3){
                        driveMecanum.moveForwardAuto(.7, -1650);
                        //driveMecanum.turn(.7, 950);
                        timer.reset();
                    }
                    if(driveMecanum.getBL().getCurrentPosition() < - 1640 && step == 3){
                        resetEnc_step();
                    }
                    if(step == 4){
                        driveMecanum.turn(.7, 1900);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() > 1790 && step == 4){
                        resetEnc_step();
                        driveMecanum.setPowerZero();
                    }
                    if(step == 5){
                        driveMecanum.right(.5, -320);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() > 310 && step == 5){
                        resetEnc_step();
                        driveMecanum.setPowerZero();
                    }
                    if(step == 6){
                        braco.pitchAuto(.6, 2000);
                    }
                    if(braco.getMotorBraco().getCurrentPosition() > 1990 && step == 6) {
                        resetEnc_step();
                    }
                    if(step == 7){
                        braco.bandeja.pitchAutoCtrl(.4, 100);
                    }
                    if(braco.bandeja.getMotorPitch().getCurrentPosition() > 90 && step == 7){
                        resetEnc_step();
                        driveMecanum.setPowerZero();
                        timer.reset();
                        timer.startTime();
                    }
                    if(step == 8){
                        braco.bandeja.destravarBandejaAuto(.5);
                        braco.bandeja.getMotorPitch().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    if(timer.milliseconds() > 800 && step == 8){
                        braco.bandeja.destravarBandejaAuto(.3);
                        braco.bandeja.pitchAutoCtrl(.4, 0);
                        braco.pitchAuto(.4, 0);
                        resetEnc_step();
                    }
                    if(braco.getMotorBraco().getCurrentPosition() <= 5 && step == 9){
                        driveMecanum.right(.6, 400);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() < - 390 && step == 9){
                        resetEnc_step();
                        driveMecanum.setPowerZero();
                    }
                    /*
                    if(driveMecanum.getBL().getCurrentPosition() <= -490 && step == 3){
                        resetEnc_step();
                    }
                    if(step == 4){
                        driveMecanum.right(.7, -1200);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() > 1190 && step == 4){
                        resetEnc_step();
                    }
                    if(step == 5){
                        driveMecanum.moveForwardAuto(.7, -1500);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() < -1490 && step == 5){
                        resetEnc_step();
                    }
                    if(step == 6) {
                        timer = new ElapsedTime();
                        timer.reset();
                        timer.startTime();
                        coletor.collectorControl(0, -0.6);
                        driveMecanum.setPowerZero();
                        resetEnc_step();
                    }
                    if(timer.seconds() > 4 && step == 7){
                        coletor.collectorControl(.6, -0.6);
                        resetEnc_step();
                    }*/
                    break;
                case RIGHT:
                    if(step == 0) driveMecanum.moveForwardAuto(-0.7, 1250);

                    if(driveMecanum.getBL().getCurrentPosition() > 1240 && step == 0) {
                        resetEnc_step();
                    }

                    if(step == 1) driveMecanum.turn(.6, -950);

                    if(driveMecanum.getBL().getCurrentPosition() < -940 && step == 1){
                        resetEnc_step();
                    }

                    if (step == 2){
                        driveMecanum.moveForwardAuto(.4, 50);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() > 45 && step == 2){
                        resetEnc_step();
                    }
                    if(step == 3){
                        braco.bandeja.setAutoZero();
                        driveMecanum.setPowerZero();
                        timer.reset();
                        timer.startTime();
                    }
                    if(braco.bandeja.getTravaAuto().getPosition() >= .4 && step == 3) {
                        resetEnc_step();
                    }

                    if(timer.milliseconds() > 400 && step == 4){
                        timer.reset();
                        timer.startTime();
                    }

                    if(driveMecanum.getBL().getPower() < .5 && step == 4){
                        resetEnc_step();
                    }

                    if(step == 5 && timer.milliseconds() > 500){
                        timer.reset();
                        driveMecanum.right(.7, 900);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() < - 890 && step == 5){
                        resetEnc_step();
                    }
                    if(step == 6){
                        driveMecanum.moveForwardAuto(.7, 2300);
                    }
                    if(driveMecanum.getBL().getCurrentPosition() > 2290 && step == 6){
                        resetEnc_step();
                    }

                    if(step == 7) {
                        timer = new ElapsedTime();
                        timer.reset();
                        timer.startTime();
                        coletor.collectorControl(0, -0.5);
                        driveMecanum.setPowerZero();
                        resetEnc_step();
                    }

                    if(step == 7 && timer.seconds() > 3){
                        coletor.collectorControl(0, 0);
                        resetEnc_step();
                    }
                    break;
                default:
                    break;
                }
            }
            telemetry.addData("timer", timer.seconds());
            telemetry.addData("getBL", driveMecanum.getBL().getCurrentPosition());
            //telemetry.addData("odom", driveMecanum.getOdomY().getCurrentPosition());
            telemetry.update();
        }

    public void resetEnc_step(){
        driveMecanum.resetEnc();
        step++;
    }

    private void initCamera() {

        // Create the AprilTag processor.

        pipelineAzul = new Pipeline_Azul();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(pipelineAzul)
                .build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(pipelineAzul, true);

    }
}
