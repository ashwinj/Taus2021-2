//Mapping and setting up harware for robot

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Hardware {

    // Motor variable names
    public DcMotorEx frontLeftMotor = null;
    public DcMotorEx frontRightMotor = null;
    public DcMotorEx backLeftMotor = null;
    public DcMotorEx backRightMotor = null;
    public Servo IndexerServo = null;//theIndexerServoThatGoesOnTheFrontOfTheRobotAndKicksRingsIntoTheShooterSoWeCanScoreRingsAndBeVeryHappy
    public Servo Blocker = null;
    public Servo clawServo = null;
    public Servo armServo = null;
    public DcMotor intake = null;
    public DcMotor intake2 = null;
    public DcMotorEx shooter = null;
    public DistanceSensor distance = null;
    public boolean encoder;
    public BNO055IMU imu; //inertial measurement unit
    public BNO055IMU imu2; //inertial measurement unit

    // Other variable names
    HardwareMap hwMap;
    public ElapsedTime period = new ElapsedTime();


    public Hardware(boolean encoder) {
        hwMap = null;
        this.encoder = encoder; //setting if you want to run with encoders
    }

    //called in initializeRobot method in AutonomousMethods
    public void initializeHardware(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;
        period.reset();

        // Define Motors
        frontLeftMotor = hwMap.get(DcMotorEx.class,"front_left");
        frontRightMotor = hwMap.get(DcMotorEx.class,"front_right");
        backLeftMotor = hwMap.get(DcMotorEx.class,"back_left");
        backRightMotor = hwMap.get(DcMotorEx.class,"back_right");
        clawServo = hwMap.servo.get("claw");
        armServo = hwMap.servo.get("arm");
        IndexerServo = hwMap.servo.get("indexer");
        Blocker = hwMap.servo.get("blocker");
        //IndexerServo.setPosition();
        intake = hwMap.dcMotor.get("intake");
        intake2 = hwMap.dcMotor.get("intake2");
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        distance = hwMap.get(DistanceSensor.class, "dist");


        // Initialize Motors

        // ******MAY CHANGE *******  Fix Forward/Reverse under testing
        Blocker.setDirection(Servo.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        if(encoder) {
            // May use RUN_USING_ENCODERS if encoders are installed
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else{
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        intake.setPower(0);
        intake2.setPower(0);
        shooter.setPower(0);



        //Define Sensors
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU; //inertial measurement unit
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; //angle unit to degrees
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false; //will log values if true
        imu.initialize(parameters); //initializing using above parameters

        while(!imu.isGyroCalibrated()){
        }

        imu2 = hwMap.get(BNO055IMU.class, "imu2");

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();

        parameters2.mode = BNO055IMU.SensorMode.IMU; //inertial measurement unit
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES; //angle unit to degrees
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.loggingEnabled = false; //will log values if true
        imu2.initialize(parameters2); //initializing using above parameters

        while(!imu2.isGyroCalibrated()){
        }

        //Define Servos
    }

}