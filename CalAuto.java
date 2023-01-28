package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="CAL_AUTO", group = "DEBUG")
public class CalAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeHardware(hardwareMap);
        waitForStart();
        robot.moveElevator(ElevatorPositions.LOW);
        robot.moveElevator(ElevatorPositions.HIGH);
        robot.moveElevator(ElevatorPositions.GROUND);
        //robot.moveAxial(20);
        //sleep(500);
        //robot.useElevatorAuto(ElevatorPositions.HIGH);
        //sleep(200);
        //robot.moveAxial(1);
        //sleep(200);
        //robot.useElevatorAuto(ElevatorPositions.GROUND);
        //sleep(200);
        //robot.rotate(160);
        //sleep(200);
        //robot.moveAxial(20);
    }
}
