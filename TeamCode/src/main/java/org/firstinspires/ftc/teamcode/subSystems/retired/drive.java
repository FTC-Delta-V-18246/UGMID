package org.firstinspires.ftc.teamcode.subSystems.retired;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.geometry.Point;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;
import org.firstinspires.ftc.teamcode.subSystems.hood;
import org.firstinspires.ftc.teamcode.utilnonrr.PIDMath;

@Config

public class drive
{

    static final double COUNTS_PER_MOTOR_REV = 537.5;    // eg: Hex HD Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    LinearOpMode opModeObj;


    //variables for taking the derivative of error
    private double lasterror;
    private double error;
    private double output;
    private double scaledoutput;
    private double nowtime, thentime;

    private double correction;
    private double avg = 0;
    private double cp;
    private double tp;

    private static double acceptableShootingAngleError = 3;


    //for gyro strafing
    public static double kPstrafeangle = 0.015;
    public static double kDstrafeangle = 0.006;

    //for gyro driving
    public static double kPdriveangle = 0.010;
    public static double kDdriveangle = 0.006;

    //for turning
    public static double kPturn = .036; //.03
    public static double kDturn = 0.003;

    private double strafeHeading = 0;
    private double currentheading;

    private int frontLefttarget, frontRighttarget,backRighttarget,backLefttarget;

    public DcMotorEx frontLeft   = null;
    public DcMotorEx frontRight  = null;
    public DcMotorEx backLeft   = null;
    public DcMotorEx backRight  = null;
    public revIMU gyroObj;
    public odometry missileObj;
    public hood hoodObj;
    private ElapsedTime runtime;
    PIDMath translation;
    PIDMath rotation;
    public static double kPT = 0, kDT = 0, kFT = -.27;
    public static double kPR = 0, kDR = 0, kFR = 0;
    public drive(LinearOpMode opMode, hardwareGenerator gen, SampleMecanumDrive driver, hood flipper, ElapsedTime timer){
        frontLeft = gen.frontLeftM;
        frontRight = gen.frontRightM;
        backLeft = gen.backLeftM;
        backRight = gen.backRightM;
        opModeObj = opMode;
        hoodObj = flipper;
        runtime = timer;
        rotation = new PIDMath(kPR,0,kDR);
    }

    public void driveX(double percentSpeed, double leftStickY, double leftStickX, double rightStickX){
        double drive = leftStickY; // inputs, may require this.opModeObj.
        double strafe = -leftStickX;
        double turn = .8*rightStickX;
        dst(percentSpeed*drive, percentSpeed*strafe, turn);
    }
    public void dst(double drive, double strafe, double turn){
        double lfP = drive + strafe - turn; // determining wheel proportions
        double lbP = drive - strafe - turn;
        double rfP = drive - strafe + turn;
        double rbP = drive + strafe + turn;

        /*
        double max = Math.max(1.0, Math.abs(lfP)); // smooth out a little - can be deleted
        max = Math.max(max, Math.abs(lbP));
        max = Math.max(max, Math.abs(rfP));
        max = Math.max(max, Math.abs(rbP));
*/

        frontLeft.setPower(lfP); // set powers
        backLeft.setPower(lbP);
        frontRight.setPower(rfP);
        backRight.setPower(rbP);
    }

    public void driveT(double leftPower, double rightPower){
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
    }

    /**
     * Code from FTCLib
     *
     * @param leftStickX   horizontal of the left stick of the main gamepad
     * @param leftStickY   vertical of the left stick of the main gamepad
     * @param rightStickX  horizontal of the right stick of the main gamepad
     * @param rightStickX    how fast it turns
     */

    public void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX, double angle){
        Vector2d initial = new Vector2d(leftStickX,leftStickY);
        initial =  initial.rotateBy(-angle);
        //double theta = Math.atan2(leftStickX,leftStickY);
        double theta = initial.angle();
        frontLeft.setPower(initial.magnitude() * Math.sin(theta + Math.PI / 4) + rightStickX);
        frontRight.setPower(initial.magnitude() * Math.sin(theta - Math.PI / 4) - rightStickX);
        backLeft.setPower(initial.magnitude() * Math.sin(theta - Math.PI / 4) + rightStickX);
        backRight.setPower(initial.magnitude() * Math.sin(theta + Math.PI / 4) - rightStickX);

    }
    public void turnToShoot(Point goalLocation, Pose2d curPose){
        driveT(rotation.calculateGain(turnToAbsolute(goalLocation,curPose),runtime.seconds()), -rotation.calculateGain(turnToAbsolute(goalLocation,curPose),runtime.seconds()));
    }
    public void turnOver(Point goalLocation, Pose2d curPose){
        if(turnToAbsolute(goalLocation,curPose)<Math.toRadians(1.5)){
            dst(0,0,0);
        }
        else{
            driveT(rotation.calculateGain(turnToAbsolute(goalLocation,curPose),runtime.seconds()), -rotation.calculateGain(turnToAbsolute(goalLocation,curPose),runtime.seconds()));
        }
    }

    private double robotAngleToTarget(Point target, Pose2d curPos){
        double tgtAngle = 3* Math.PI/2.0 - Math.atan2(target.x-curPos.getX(),target.y-curPos.getY());
        //opModeObj.telemetry.addData("Target Angle", tgtAngle);
        return tgtAngle;
    }
    private double turnToAbsolute(Point target, Pose2d curPos){
        double tgtAngle = robotAngleToTarget(target, curPos);
        if(Math.abs(tgtAngle-curPos.getHeading())< Math.abs(-2* Math.PI+tgtAngle-curPos.getHeading())){
            return tgtAngle-curPos.getHeading();
        }
        else{
            return -2* Math.PI+tgtAngle-curPos.getHeading();
        }
        //return tgtAngle - curPos.getHeading();
    }
    private double turnToAbsolute(double tgtAngle, Pose2d curPos){
        if(Math.abs(tgtAngle-curPos.getHeading())< Math.abs(-2* Math.PI+tgtAngle-curPos.getHeading())){
            return tgtAngle-curPos.getHeading();
        }
        else{
            return -2* Math.PI+tgtAngle-curPos.getHeading();
        }
        //return tgtAngle - curPos.getHeading();
    }

}
