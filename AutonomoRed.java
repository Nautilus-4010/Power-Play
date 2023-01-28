package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

@Autonomous(name="Red alliance")
public class AutonomoRed extends LinearOpMode {
    private final RobotHardware robot = new RobotHardware(this);
    private OpenGLMatrix targetPose;


    @Override public void runOpMode() {
        robot.initializeHardware(hardwareMap);
        robot.initVuforia();
        robot.startTracking();


        waitForStart();
        while (opModeIsActive()) {
            robot.followTarget();
            telemetry.update();
        }
    }
}