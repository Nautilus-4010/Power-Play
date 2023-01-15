package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class RobotHardware {
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;

    private LinearOpMode opMode;

    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initializeHardware(HardwareMap hardwareMap) {
        opMode.telemetry.addData("Status", "Initializing...");
        opMode.telemetry.update();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();
    }

    public void logChasisPowers() {
        opMode.telemetry.addData("LF", leftFrontDrive.getPower());
        opMode.telemetry.addData("RF", rightFrontDrive.getPower());
        opMode.telemetry.addData("LB", leftBackDrive.getPower());
        opMode.telemetry.addData("RB", rightBackDrive.getPower());
    }
    
    public void resetEncoders() {
        setChasisRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setChasisRunMode(DcMotor.RunMode runmode) {
        leftFrontDrive.setMode(runmode);
        leftBackDrive.setMode(runmode);
        rightFrontDrive.setMode(runmode);
        rightBackDrive.setMode(runmode);
    }
}