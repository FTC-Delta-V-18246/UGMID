package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


public class hardwareGenerator {
    public DcMotorEx frontLeftM   = null;
    public DcMotorEx frontRightM  = null;
    public DcMotorEx backLeftM   = null;
    public DcMotorEx backRightM  = null;
    public DcMotorEx flyWheelM   = null;
    public DcMotorEx flyWheelM1 = null;
    public DcMotorEx outerRollerMI = null;
    public DcMotorEx outerRollerMII = null;
    public Servo pivotServoL= null;
    public Servo pivotServoR = null;
    public Servo intakeServoL = null;
    public Servo intakeServoR = null;
    public Servo pusherServo = null;
    public Servo wobblePivot = null;
    public Servo wobbleClaw = null;
    public BNO055IMU imu = null;
    public DigitalChannel shooterLimitSwitch = null;
   // public Encoder leftO, rightO, strafeO;
    public double indexAngle = .43;

    private LinearOpMode opModeObj;

    public hardwareGenerator(LinearOpMode opMode){
        opModeObj = opMode;
        frontLeftM  = opModeObj.hardwareMap.get(DcMotorEx.class,"front_left");
        frontRightM = opModeObj.hardwareMap.get(DcMotorEx.class,"front_right");
        backLeftM = opModeObj.hardwareMap.get(DcMotorEx.class,"back_left");
        backRightM = opModeObj.hardwareMap.get(DcMotorEx.class,"back_right");
        frontLeftM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRightM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeftM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRightM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRightM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeftM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRightM.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //backLeftM.setDirection(DcMotorEx.Direction.REVERSE);
        //frontLeftM.setDirection(DcMotorEx.Direction.REVERSE);
        backRightM.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightM.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Set all motors to zero power
        frontLeftM.setPower(0);
        frontRightM.setPower(0);
        backLeftM.setPower(0);
        backRightM.setPower(0);

        intakeServoL = opModeObj.hardwareMap.get(Servo.class, "intake_servoL");
        intakeServoR = opModeObj.hardwareMap.get(Servo.class, "intake_servoR");

        //intakeServoL.setDirection(Servo.Direction.REVERSE);
        intakeServoR.setDirection(Servo.Direction.REVERSE);
        outerRollerMI = opModeObj.hardwareMap.get(DcMotorEx.class, "outer_roller");
        outerRollerMII = opModeObj.hardwareMap.get(DcMotorEx.class, "outer_roller1");
        outerRollerMI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outerRollerMII.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outerRollerMI.setDirection(DcMotorEx.Direction.FORWARD);
        outerRollerMII.setDirection(DcMotorEx.Direction.REVERSE);
        outerRollerMII.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outerRollerMI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wobblePivot = opModeObj.hardwareMap.get(Servo.class, "wobble_pivot");
        wobblePivot.setDirection(Servo.Direction.REVERSE);
        wobbleClaw = opModeObj.hardwareMap.get(Servo.class,"wobble_claw");
        //wobbleClaw.setDirection(Servo.Direction.REVERSE);


        flyWheelM = opModeObj.hardwareMap.get(DcMotorEx.class, "fly_wheel");
        flyWheelM1 = opModeObj.hardwareMap.get(DcMotorEx.class,"notaMotor");
        flyWheelM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // flyWheelM1.setDirection(DcMotorEx.Direction.);

        pivotServoL = opModeObj.hardwareMap.get(Servo.class,"pivot_left");
        pivotServoR = opModeObj.hardwareMap.get(Servo.class,"pivot_right");
        pivotServoR.setDirection(Servo.Direction.FORWARD);

        pusherServo = opModeObj.hardwareMap.get(Servo.class,"pusher");

        shooterLimitSwitch = opModeObj.hardwareMap.get(DigitalChannel.class, "sensor_digital");
        shooterLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        imu = opModeObj.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);






    }
}
