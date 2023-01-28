package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

public class RobotHardware {
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public Servo intake;
    public DcMotor elevatorRight, elevatorLeft;
    public AnalogInput potentiometer;

    final double ELEVATOR_RISE_POWER = 0.5;
    final double ELEVATOR_LOWER_POWER = -ELEVATOR_RISE_POWER;
    final double MOTOR_UPDATE_PERIOD_MS = 50;
    final double MOTOR_POWER_INCREMENT = 0.065;

    private ElapsedTime timer = new ElapsedTime();
    private double lastMotorUpdateTime;
    private LinearOpMode opMode;
    private WebcamName webcamName;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private double lastTracakbleSearchTime;
    private TargetInfo identifiedTrackable;
    private boolean findTrackable = false;

    public RobotHardware(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    private void stopChassis(){
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
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
        lastMotorUpdateTime = timer.milliseconds();
        lastTracakbleSearchTime = lastMotorUpdateTime;

        initializeMechanisms(hardwareMap);
        initializeSensors(hardwareMap);

        opMode.telemetry.addData("Status", "Initialized");
        opMode.telemetry.update();
    }

    public void initializeMechanisms(HardwareMap hardwareMap) {
        elevatorRight = hardwareMap.get(DcMotor.class, "elevator_right");
        elevatorLeft = hardwareMap.get(DcMotor.class, "elevator_left");
        elevatorRight.setDirection(DcMotor.Direction.FORWARD);
        elevatorLeft.setDirection(DcMotor.Direction.REVERSE);
        intake = hardwareMap.get(Servo.class, "intake");
        // In case the suspenders move the elevator
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
    }
    public void initializeSensors(HardwareMap hardwareMap){
        potentiometer = hardwareMap.get(AnalogInput.class, "pot");
        initImu();
    }

    // ******************************************
    // *                 CHASIS                 *
    // ******************************************

    public void move(double drive, double lateral, double yaw) {
        double currentTime = timer.milliseconds();
        if (currentTime - lastMotorUpdateTime > MOTOR_UPDATE_PERIOD_MS) {
            double leftFrontTarget  = drive + lateral + yaw;
            double rightFrontTarget = drive - lateral - yaw;
            double leftBackTarget   = drive - lateral + yaw;
            double rightBackTarget  = drive + lateral - yaw;
            // Normalise velocities
            double max = Math.max(Math.abs(leftFrontTarget), Math.abs(rightFrontTarget));
            max = Math.max(max, Math.abs(leftBackTarget));
            max = Math.max(max, Math.abs(rightBackTarget));
            if (max > 1.0) {
                leftFrontTarget  /= max;
                rightFrontTarget /= max;
                leftBackTarget   /= max;
                rightBackTarget  /= max;
            }
            // Ramp velocities
            double leftFrontPower  = getIncreasedPower(leftFrontDrive.getPower(), leftFrontTarget);
            double rightFrontPower = getIncreasedPower(rightFrontDrive.getPower(), rightFrontTarget);
            double leftBackPower   = getIncreasedPower(leftBackDrive.getPower(), leftBackTarget);
            double rightBackPower  = getIncreasedPower(rightBackDrive.getPower(), rightBackTarget);
            // Update velocities
            leftFrontDrive.setPower(leftFrontTarget);
            rightFrontDrive.setPower(rightFrontTarget);
            leftBackDrive.setPower(leftBackTarget);
            rightBackDrive.setPower(rightBackTarget);
            // opMode.telemetry.addData("LF", leftFrontTarget + " " + leftFrontPower);
            // opMode.telemetry.addData("RF", rightFrontTarget + " " + rightFrontPower);
            // opMode.telemetry.addData("LB", leftBackTarget + " " + leftBackPower);
            // opMode.telemetry.addData("RB", rightBackTarget + " " + rightBackPower);
            lastMotorUpdateTime = currentTime;
        }
    }

    private double getIncreasedPower(double current, double target) {
        double diff = target - current;
        if(diff == 0)
            return current;
        double absDiff = Math.abs(diff);
        double sign = diff / absDiff;
        double valueToIncrement = sign * Math.min(absDiff, MOTOR_POWER_INCREMENT);
        return current + valueToIncrement;
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

    // ******************************************
    // *                 INTAKE                 *
    // ******************************************

    public void openIntake() {
        double OPEN_POSITION = 0.45;
        intake.setPosition(OPEN_POSITION);
    }

    public void closeIntake() {
        double CLOSE_POSITION = 0;
        intake.setPosition(CLOSE_POSITION);
    }

    // ********************
    // *     ELEVATOR     *
    // ********************
    public void moveElevator(double power) {
        elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorRight.setPower(power);
        elevatorLeft.setPower(power);
    }
    public void showElevatorTicks(){
        opMode.telemetry.addData("Left ticks", elevatorLeft.getCurrentPosition());
        opMode.telemetry.addData("Right ticks", elevatorRight.getCurrentPosition());
    }
    public void showPotVoltage(){
        opMode.telemetry.addData("Pot voltage", potentiometer.getVoltage());
    }
    public int moveToPosition(ElevatorPositions positions, double previousError){
        int error = 0;
        if(positions != null) {
            elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorLeft.setTargetPosition(positions.TICKS_LEFT);
            elevatorRight.setTargetPosition(positions.TICKS_RIGHT);
            if(elevatorLeft.getCurrentPosition() < elevatorLeft.getTargetPosition()){
                elevatorLeft.setPower(ELEVATOR_RISE_POWER);
                elevatorRight.setPower(ELEVATOR_RISE_POWER);
            } else{
                elevatorLeft.setPower(ELEVATOR_LOWER_POWER);
                elevatorRight.setPower(ELEVATOR_LOWER_POWER);
            }
        }
        return error;
    }

    public double usePot(ElevatorPositions positions, double previousError){
        if(positions != null) {
            PID potPID = new PID(5, 0, 0.01);
            double targetVoltage = positions.VOLTAGE,
                    currentVoltage = potentiometer.getVoltage();
            double error = targetVoltage - currentVoltage;
            double corrections = potPID.correctionValue(currentVoltage, targetVoltage, previousError);
            opMode.telemetry.addData("Values", "Error: (%.3f) Target: (%.3f) Current: (%.3f)",
                    error, targetVoltage, currentVoltage);
            opMode.telemetry.addLine("Correction: " + corrections);
            moveElevator(Range.clip(corrections, ELEVATOR_LOWER_POWER, ELEVATOR_RISE_POWER));
            return error;
        }
        return 0;
    }
    public void useElevatorAuto(ElevatorPositions positions){
        double error = usePot(positions, 0);
        while(opMode.opModeIsActive() &&
                Math.abs(positions.VOLTAGE - potentiometer.getVoltage()) < 0.005){
            error = usePot(positions, error);
        }
    }

    // ******************************************
    // *                 VISION                 *
    // ******************************************
    public void initVuforia() {
        webcamName = opMode.hardwareMap.get(WebcamName.class, "camara");
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATjrkEL/////AAABmfR2/BPftkOFvL9kl5ElbHswfU6Tuno4QSB4aHpVUmWaWqKdEUps2CsnGbmjoGqMAfOjyPlhrew8njlemEsarH9XKySF9i0egaUhOiT2fE0MivatYaT037ZwPe1bOkI1GGmd2CsWL8GeupcT91XQkGhRcMyTS3ZfmDYu1/HmcRxCy4zxwbiyPVcoHtsh+KPfjI29mv9YfMStiB4/o8FgefPbTGtX6L9zeoyUemNIMN1WcaMi6wSM7rB7kF3VnUJCrXAca6YmFNEr6GEdJX4G7JhO5EiD6K/e1+wZ0fLtWiQDWe09Bgxxpp2n+qHeccA06zA8nNTo2F07UORoM40ZK29vMj4eh0GjyNMAOmWcuQeI";
        parameters.useExtendedTracking = false;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {

    }

    public void logIdentifiedTarget() {
        TargetInfo trackable = getVisibleTrackable();
        logIdentifiedTarget(trackable);
    }

    public void logIdentifiedTarget(TargetInfo trackable) {
        if (trackable == null)
            opMode.telemetry.addData("Trackable:", "No visible target");
        else {
            opMode.telemetry.addData("Trackable:", trackable.name);
            opMode.telemetry.addData("Trackable:", "X: (%.2f)  Y: (%.2f)  Z: (%.2f)", trackable.x, trackable.y, trackable.z);
        }
    }

    public void followTarget() {
        final double MM_PER_INCH = 25.40;
        TargetInfo trackable = getVisibleTrackable();
        if (trackable != null){
            double targetX = trackable.x;
            double targetY = trackable.z;
            final double DESIRED_DISTANCE = MM_PER_INCH * 9;
            final double SPEED_GAIN =   0.0012 ;
            final double TURN_GAIN  =   0.0018 ;
            double targetRange = Math.hypot(targetX, targetY);
            double targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));
            double  rangeError   = (targetRange - DESIRED_DISTANCE);
            double  headingError = targetBearing;
            double drive = rangeError * SPEED_GAIN;
            double turn  = headingError * TURN_GAIN;
            logIdentifiedTarget(trackable);
            move(drive, 0, turn);
            opMode.telemetry.addData("Range error:", rangeError);
            opMode.telemetry.addData("Heading error", headingError);
            opMode.telemetry.addData("Drive:", drive);
            opMode.telemetry.addData("Turn", turn);
        } else {
            move(0, 0, 0);
        }
    }

    public void startTracking() {
        trackables = this.vuforia.loadTrackablesFromAsset("PowerPlay");
        trackables.get(0).setName("Red Audience Wall");
        trackables.get(1).setName("Red Rear Wall");
        trackables.get(2).setName("Blue Audience Wall");
        trackables.get(3).setName("Blue Rear Wall");
        trackables.activate();
    }

    private TargetInfo getVisibleTrackable() {
        double currentTime = timer.milliseconds();
        if(currentTime - lastTracakbleSearchTime > 100) {
            lastTracakbleSearchTime = currentTime;
            identifiedTrackable = null;
            for (VuforiaTrackable trackable : trackables) {
                VuforiaTrackableDefaultListener targetListener = ((VuforiaTrackableDefaultListener) trackable.getListener());
                boolean isTrackableVisible = targetListener.isVisible();
                if(isTrackableVisible){
                    identifiedTrackable = new TargetInfo(trackable);
                    break;
                }
            }
        }
        return identifiedTrackable;
    }
    // **********************
    // *     AUTONOMOUS     *
    // **********************
    private static final int ticksPerCM_Axial = 25,
            ticksPerCM_Lateral = 30;
    private static final double BASE_SPEED_AUTONOMOUS = 0.5,
    MAX_SPEED_AUTONOMOUS = 0.6;
    public BNO055IMU imu;
    Orientation lastMeasuredAngle;
    double globalAngle;
    final double PROPORTIONAL = 0.005; // No time for PID
    public void initImu(){
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = true;
        params.loggingTag = "IMU_DEBUG";
        imu.initialize(params);
        lastMeasuredAngle = new Orientation();
    }
    public void resetAngle(){
        lastMeasuredAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        double differenceAngle = angles.firstAngle - lastMeasuredAngle.firstAngle;
        if(differenceAngle < -180) differenceAngle += 360;
        else if (differenceAngle > 180) differenceAngle -= 360;
        globalAngle += differenceAngle;
        lastMeasuredAngle = angles;
        return globalAngle;
    }

    public void moveAxial(double distanceInCM){
        resetEncoders();
        double desiredPosition = getAngle();
        double leftPower = 0, rightPower = 0;
        int targetPosition = (int) Math.round(distanceInCM * ticksPerCM_Lateral);
        leftFrontDrive.setTargetPosition(targetPosition);
        rightFrontDrive.setTargetPosition(targetPosition);
        leftBackDrive.setTargetPosition(targetPosition);
        rightBackDrive.setTargetPosition(targetPosition);
        setChasisRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opMode.opModeIsActive() && leftFrontDrive.isBusy() && leftBackDrive.isBusy()
                && rightBackDrive.isBusy() && rightFrontDrive.isBusy()){
            double error = getAngle(), relativeError;
            relativeError = (desiredPosition - error) / desiredPosition;
            if(distanceInCM > 0){
                rightPower = leftPower = BASE_SPEED_AUTONOMOUS;
                leftPower -= leftPower * relativeError * PROPORTIONAL;
                rightPower += rightPower * relativeError * PROPORTIONAL;
            } else if(distanceInCM < 0){
                rightPower = leftPower = -BASE_SPEED_AUTONOMOUS;
                leftPower += leftPower * relativeError * PROPORTIONAL;
                rightPower -= rightPower * relativeError * PROPORTIONAL;
            }
            leftPower = Range.clip(leftPower, -MAX_SPEED_AUTONOMOUS, MAX_SPEED_AUTONOMOUS);
            rightPower = Range.clip(rightPower, -MAX_SPEED_AUTONOMOUS, MAX_SPEED_AUTONOMOUS);
            opMode.telemetry.addData("Left power: ", leftPower);
            opMode.telemetry.addData("Right power: ", rightPower);
            opMode.telemetry.addData("Error: ", relativeError);
            opMode.telemetry.addData("Desviacion: ", error);
            opMode.telemetry.addData("Target ", desiredPosition);
            rightFrontDrive.setPower(rightPower);
            leftBackDrive.setPower(leftPower);
            leftFrontDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
        }
        stopChassis();
        setChasisRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveLateral(double distanceInCM){
        resetEncoders();
        double desiredPosition = getAngle();
        double leftPower = 0, rightPower = 0;
        int targetPosition = (int) Math.round(distanceInCM * ticksPerCM_Axial);
        leftFrontDrive.setTargetPosition(targetPosition);
        rightFrontDrive.setTargetPosition(-targetPosition);
        leftBackDrive.setTargetPosition(-targetPosition);
        rightBackDrive.setTargetPosition(targetPosition);
        setChasisRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opMode.opModeIsActive() && leftFrontDrive.isBusy() && leftBackDrive.isBusy()
                && rightBackDrive.isBusy() && rightFrontDrive.isBusy()){
            double error = getAngle(), relativeError;
            relativeError = (desiredPosition - error) / desiredPosition;
            if(distanceInCM > 0){
                rightPower = leftPower = BASE_SPEED_AUTONOMOUS + 0.3;
                leftPower -= leftPower * relativeError * PROPORTIONAL;
                rightPower += rightPower * relativeError * PROPORTIONAL;
            } else if(distanceInCM < 0){
                rightPower = leftPower = -BASE_SPEED_AUTONOMOUS;
                leftPower += leftPower * relativeError * PROPORTIONAL;
                rightPower -= rightPower * relativeError * PROPORTIONAL;
            }
            leftPower = Range.clip(leftPower, -MAX_SPEED_AUTONOMOUS - 0.3, MAX_SPEED_AUTONOMOUS + 0.3);
            rightPower = Range.clip(rightPower, -MAX_SPEED_AUTONOMOUS - 0.3, MAX_SPEED_AUTONOMOUS + 0.3);
            opMode.telemetry.addData("Left power: ", leftPower);
            opMode.telemetry.addData("Right power: ", rightPower);
            opMode.telemetry.addData("Error: ", relativeError);
            opMode.telemetry.addData("Desviacion: ", error);
            opMode.telemetry.addData("Target ", desiredPosition);
            rightFrontDrive.setPower(rightPower);
            leftBackDrive.setPower(leftPower);
            leftFrontDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
        }
        stopChassis();
        setChasisRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void rotate(double degrees){
        double leftPower, rightPower;
        resetAngle();
        if(degrees < 0){
            leftPower = BASE_SPEED_AUTONOMOUS;
            rightPower = -BASE_SPEED_AUTONOMOUS;
        }
        else if(degrees > 0){
            leftPower = -BASE_SPEED_AUTONOMOUS;
            rightPower = BASE_SPEED_AUTONOMOUS;
        }
        else return;
        rightFrontDrive.setPower(rightPower);
        leftBackDrive.setPower(leftPower);
        leftFrontDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() >= 0) {
                opMode.sleep(100);
            }

             while (opMode.opModeIsActive() && getAngle() > degrees) {
                 opMode.sleep(100);
             }
        }
        else {    // left turn.{
            while (opMode.opModeIsActive() && getAngle() < degrees) {
                opMode.sleep(100);
            }
        }
        stopChassis();
        resetAngle();
    }
}
enum ElevatorPositions{
    // TICS measured from ground level
    GROUND(0.85, 0, 0),
    LOW(0.62, 29, 25),
    MEDIUM(0.41, 50, 47),
    HIGH(0.065, 78, 71);
    public final double VOLTAGE;
    public final int TICKS_LEFT, TICKS_RIGHT;
    ElevatorPositions(double voltage, int ticksLeft, int ticksRight){
        VOLTAGE = voltage;
        TICKS_LEFT = ticksLeft;
        TICKS_RIGHT = ticksRight;
    }
}
