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

        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");

        waitForStart();

        while (opModeIsActive()) {

            backRightDrive.setPower(.5);
            backLeftDrive.setPower(.5);
            frontRightDrive.setPower(.5);
            frontLeftDrive.setPower(.5);

            telemetry.addData("Encoder BackRight", backRightDrive.getCurrentPosition());
            telemetry.update();

            sleep(5000);

            backRightDrive.setPower(-0.5);
            frontRightDrive.setPower(-0.5);
            backLeftDrive.setPower(0.5);
            frontLeftDrive.setPower(0.5);

            telemetry.addData("Encoder FrontRight", frontRightDrive.getCurrentPosition());
            telemetry.update();

            sleep(3000);

            backRightDrive.setPower(.8);
            backLeftDrive.setPower(.8);
            frontRightDrive.setPower(.8);
            frontLeftDrive.setPower(.8);

            telemetry.addData("Encoder RightBack", backRightDrive.getCurrentPosition());
            telemetry.update();

            sleep(4000);

            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontLeftDrive.setPower(0);

            telemetry.addData("Encoder RightBack", backRightDrive.getCurrentPosition());
            telemetry.update();

            sleep(1000);

            backRightDrive.setPower(1);
            backLeftDrive.setPower(-1);
            frontRightDrive.setPower(-1);
            frontLeftDrive.setPower(1);

            telemetry.addData("Encoder RightBack", backRightDrive.getCurrentPosition());
            telemetry.update();

            sleep(800);

            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontLeftDrive.setPower(0);

            telemetry.addData("Encoder RightBack", backRightDrive.getCurrentPosition());
            telemetry.update();

            sleep(600);

            backRightDrive.setPower(.7);
            backLeftDrive.setPower(.7);
            frontRightDrive.setPower(.7);
            frontLeftDrive.setPower(.7);

            telemetry.addData("Encoder RightBack", backRightDrive.getCurrentPosition());
            telemetry.update();

            sleep(450);

            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontLeftDrive.setPower(0);

            telemetry.addData("Encoder RightBack", backRightDrive.getCurrentPosition());
            telemetry.update();

            sleep(500);

        }
    }
}

