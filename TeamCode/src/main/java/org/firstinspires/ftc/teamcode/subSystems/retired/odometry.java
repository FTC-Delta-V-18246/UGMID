package org.firstinspires.ftc.teamcode.subSystems.retired;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.geometry.Point;
import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;

@Config
public class odometry {
    LinearOpMode opModeObj;
    //public Encoder leftDeadWheel, rightDeadWheel, strafeDeadWheel;

    private double leftDeadWheelPrev = 0;
    private double rightDeadWheelPrev = 0;
    private double strafeDeadWheelPrev = 0;

    private double leftMMPerTick = 35* Math.PI/8192, rightMMPerTick  = 35* Math.PI/8192, strafeMMPerTick  = 35* Math.PI/8192;

    private double  leftWheelDelta;
    private double rightWheelDelta;
    private double strafeWheelDelta;

    public double leftWheelTotal;
    public double rightWheelTotal;
    public double strafeWheelTotal;
    private double Xo, Yo, To;

    public double globalPositionX = 0;
    public double globalPositionY = 0;
    public double globalPositionTheta = 0;

    private double xDelta;
    private double yDelta;
    private double deltaTheta;

    private double sinTerm;
    private double cosTerm;

    private double move, strafe;

    private double cTerm, sTerm;

    public Point robotLocation;
    private double theta;

    private double cpr = 8192;
    private double wheelDiameter = 35;

    public static double trackwidth = 373.8433227;
    public static double thetaConstant = 0; //-92.94;
    public static double XMultiplier = 1.00837761;
    public static double YMultiplier = 1.00514046;
    private hardwareGenerator gen;

    public odometry(LinearOpMode opmode, hardwareGenerator gen) {
        opModeObj = opmode;
        //leftDeadWheel = new Encoder(gen.frontLeftM);
       // rightDeadWheel = new Encoder(gen.frontRightM);
       // rightDeadWheel.setDirection(Encoder.Direction.REVERSE);
       // strafeDeadWheel = new Encoder(gen.backLeftM);
        robotLocation = new Point(0,0);
        this.gen = gen;
    }

    /**
     *
     * @param leftDeadWheelCurrent current position of the left deadwheel, to be passed from the opmode to allow for bulk reads
     * @param rightDeadWheelCurrent current position of the right deadwheel, to be passed from the opmode to allow for bulk reads
     * @param strafeDeadWheelCurrent current position of the strafe deadwheel, to be passed from the opmode to allow for bulk reads
     */

    public void UpdateGlobalPosition(int leftDeadWheelCurrent,  int rightDeadWheelCurrent, int strafeDeadWheelCurrent) {
        leftWheelDelta = (leftDeadWheelCurrent-leftDeadWheelPrev)*leftMMPerTick*XMultiplier;
        rightWheelDelta = (rightDeadWheelCurrent-rightDeadWheelPrev)*rightMMPerTick*XMultiplier;
        strafeWheelDelta = (strafeDeadWheelCurrent-strafeDeadWheelPrev)*strafeMMPerTick*YMultiplier;



        //change in angle since the previous update
        deltaTheta = (rightWheelDelta-leftWheelDelta)/(trackwidth);

        // absolute angle
        leftWheelTotal = leftDeadWheelCurrent*leftMMPerTick;
        rightWheelTotal = rightDeadWheelCurrent*rightMMPerTick;
        strafeWheelTotal = strafeDeadWheelCurrent*strafeMMPerTick;


        //changes in x and y movement
       xDelta = strafeWheelDelta-thetaConstant*deltaTheta;
       yDelta = (leftWheelDelta+rightWheelDelta)/2.0;
//62.8318

        //update theta AFTER rotation matrix has been multiplied in
        globalPositionTheta = (rightWheelTotal-leftWheelTotal)/trackwidth;

        if(Math.abs(deltaTheta) < (Math.pow(10,-6))){
            sinTerm = 1-(deltaTheta*deltaTheta)/6.0;
            cosTerm = deltaTheta/2.0;
        }else{
            sinTerm = Math.sin(deltaTheta)/deltaTheta;
            cosTerm = (1- Math.cos(deltaTheta))/deltaTheta;
        }
        move = sinTerm*yDelta - cosTerm*xDelta;
        strafe = cosTerm*yDelta + sinTerm*xDelta;

        //check to see if globalPositionTheta should be the previous angle and/or wrapped
        cTerm = Math.cos(theta);
        sTerm = Math.sin(theta);
        globalPositionY += move*cTerm-strafe*sTerm;
        globalPositionX += move*sTerm+strafe*cTerm;

        /*
        //product of the rotation matrix and the [x,y] matrices
        globalPositionX+=fwdMovement*cos(globalPositionTheta)-perpMovement*sin(globalPositionTheta);
        globalPositionY+=fwdMovement*sin(globalPositionTheta)+perpMovement*cos(globalPositionTheta);
        */

        //storing deadwheel encoder values for future use
        leftDeadWheelPrev = leftDeadWheelCurrent;
        rightDeadWheelPrev = rightDeadWheelCurrent;
        strafeDeadWheelPrev = strafeDeadWheelCurrent;
        theta = globalPositionTheta;

        robotLocation = new Point(globalPositionX,globalPositionY);
    }
    public void reset(){
        leftWheelDelta = 0;
        rightWheelDelta = 0;
        strafeWheelDelta = 0;

        gen.frontLeftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gen.frontRightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gen.backLeftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gen.frontLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gen.frontRightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gen.backLeftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftWheelTotal = 0;
        rightWheelTotal = 0;
        strafeWheelTotal = 0;
        deltaTheta = 0;
        xDelta = 0;
        yDelta = 0;
        globalPositionX = 0;
        globalPositionY = 0;
        globalPositionTheta = 0;
        leftDeadWheelPrev = 0;
        rightDeadWheelPrev = 0;
        strafeDeadWheelPrev = 0;
    }
    public void setStartPosition(Pose2d point){
        Yo = point.getY();
        Xo = point.getX();
        To = point.getHeading();
       // robotLocation = new Point(globalPositionX, globalPositionY);
    }
    public void setGlobalPosition(Pose2d point){
        globalPositionY = point.getX()+Yo;
        globalPositionX = -point.getY()+Xo;
        globalPositionTheta = point.getHeading()+To;
        robotLocation = new Point(globalPositionX,globalPositionY);
    }
}

