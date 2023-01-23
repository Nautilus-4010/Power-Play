package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Teleoperado")
public class Teleoperado extends LinearOpMode {
    private RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        robot.initializeHardware(hardwareMap);

        waitForStart();
        boolean wasClicked = false, isIntakeClosed = true;
        double state = 0;

        while (opModeIsActive()) {
            // **************************
            // *     CONTROL CHASIS     *
            // **************************
            double powerMultiplier = 0.6;
            double turnSensitivity = 0.8;
            if(gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0)
                powerMultiplier = 0.9;
            else if(gamepad1.right_trigger > 0)
                powerMultiplier *= 0.5;
            double axial   = -gamepad1.left_stick_y * powerMultiplier;
            double lateral =  gamepad1.left_stick_x * powerMultiplier;
            double yaw     =  gamepad1.right_stick_x * powerMultiplier * turnSensitivity;
            robot.move(axial, lateral, yaw);
            // ****************************
            // *     CONTROL ELEVADOR     *
            // ****************************
            if(gamepad2.right_trigger > 0)
                robot.moveElevator(robot.ELEVATOR_RISE_POWER);
            else if(gamepad2.left_trigger > 0)
                robot.moveElevator(robot.ELEVATOR_LOWER_POWER);
            else
                robot.moveElevator(0);
            // **************************
            // *     CONTROL INTAKE     *
            // **************************
            if(!gamepad2.a && wasClicked)
                isIntakeClosed =! isIntakeClosed;
            wasClicked = gamepad2.a;
            if(isIntakeClosed)
                robot.closeIntake();
            else
                robot.openIntake();
            telemetry.update();
            // ***************************
            // *     CONTROL CHANNEL     *
            // ***************************
            if(gamepad2.dpad_up)
                robot.channelUp();
            else if(gamepad2.dpad_down)
                robot.initialPositionChannel();
        }
    }

    private void getConnectionInfo(){
        telemetry.addLine(robot.leftFrontDrive.getConnectionInfo());
        telemetry.addLine(robot.rightFrontDrive.getConnectionInfo());
        telemetry.addLine(robot.leftBackDrive.getConnectionInfo());
        telemetry.addLine(robot.rightBackDrive.getConnectionInfo());
    }
    private void motorInfo(){
        telemetry.addData("LF", robot.leftFrontDrive.getPower()) ;
        telemetry.addData("RF", robot.rightFrontDrive.getPower());
        telemetry.addData("LB", robot.leftBackDrive.getPower());
        telemetry.addData("RB", robot.rightBackDrive.getPower());
    }
    private void gamepadStatus(){
        telemetry.addData("LeftStick_x: %.2f", gamepad1.left_stick_x);
        telemetry.addData("LeftStick_y: %.2f", gamepad1.left_stick_y);
        telemetry.addData("RightStick_x: %.2f", gamepad1.right_stick_x);
    }
}