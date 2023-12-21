package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoSimulador", group = "Robot")
//@Disabled

public class AutoSimulador extends LinearOpMode {

    DcMotor backLeftDrive, backRightDrive, frontLeftDrive, frontRightDrive;

    @Override
    public void runOpMode() {

        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Encoder BackRight", backRightDrive.getCurrentPosition());
            telemetry.update();

            backRightDrive.setPower(.5);
            backLeftDrive.setPower(.5);
            frontRightDrive.setPower(.5);
            frontLeftDrive.setPower(.5);

            sleep(5000);

            backRightDrive.setPower(-0.5);
            frontRightDrive.setPower(-0.5);
            backLeftDrive.setPower(0.5);
            frontLeftDrive.setPower(0.5);

            sleep(3000);

            backRightDrive.setPower(.8);
            backLeftDrive.setPower(.8);
            frontRightDrive.setPower(.8);
            frontLeftDrive.setPower(.8);

            sleep(4000);

            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontLeftDrive.setPower(0);

            sleep(1000);

            backRightDrive.setPower(1);
            backLeftDrive.setPower(-1);
            frontRightDrive.setPower(-1);
            frontLeftDrive.setPower(1);

            sleep(800);

            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontLeftDrive.setPower(0);

            sleep(600);

            backRightDrive.setPower(.7);
            backLeftDrive.setPower(.7);
            frontRightDrive.setPower(.7);
            frontLeftDrive.setPower(.7);

            sleep(450);

            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontLeftDrive.setPower(0);

            sleep(500);

        }
    }
}

