package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Tracker;

import org.firstinspires.ftc.teamcode.geometry.Point;
import org.firstinspires.ftc.teamcode.utilnonrr.FFFBMath;
import org.firstinspires.ftc.teamcode.utilnonrr.PIDMath;

import java.lang.annotation.Target;

@Config
public class hood {
        LinearOpMode opModeObj;
        tracker magTrak;
        vision high;
        public static double goalVelo = 22;
        private DcMotorEx flyWheel1 = null;
        private DcMotorEx flyWheel2 = null;
        private Servo pusher;
        private Servo flap;
        private Servo lift;
        public static double leftPusherPos = .5, rightPusherPos = .37; //.3
        public static double kP = .6,kD = 0;
        public static double kV = .03, kS = 0;
        public static double kPT, kDT;
        public static double interval = 150, rinterval = 100; // minimum of 45 (realistically 55)
        public static double veloRange = 2; //max of 3, probably could be increased if we increased rinterval
        public static double flapH = .235, flapHB = .24; //24
        public static double lowerFlap = .18, highFlap = .5, levelFlap = .22;
        public static double lowerLift = .2, highLift = .84;
        public double shots = 0;
        public boolean save = false;
        boolean retracted = true;
        public static boolean done = false;
        ElapsedTime shooterTime = new ElapsedTime();
        public ElapsedTime retractTime = new ElapsedTime();
        private PIDMath flyWheel;
        private FFFBMath flyWheelf;
        private PIDMath highGoal;
        public hood(LinearOpMode opMode, hardwareGenerator gen, tracker magTrak, vision camera){
            opModeObj = opMode;
            high = camera;
            this.magTrak = magTrak;
            highGoal = new PIDMath(kPT, 0, kDT);
            flyWheel = new PIDMath(kP, 0 , kD);
            flyWheelf = new FFFBMath(kV, 0, kS);
            flyWheel1 = gen.flyWheelM;
            flyWheel2 = gen.flyWheelM1;
            pusher = gen.pusherServo;
            flap = gen.flapServo;
            lift = gen.liftServo;
            shooterTime.reset();
            shots =0;
        }
        public double calculateTargetShooterAngle(Point target, Pose2d robot, boolean power){
            double distance = Math.sqrt(Math.pow(target.x-robot.getX(),2)+ Math.pow(target.y-robot.getY(),2));
            opModeObj.telemetry.addData("Distance",distance);
           /*
            if(!power){
                return shooterHeight;//5.5357014*Math.pow(10,-5)*Math.pow(distance,2)-.0106856205*distance+.7315347531;//shooterHeight;//
            }else{
                return shooterHeightP;
            }

            */
            if(!power) {
                goalVelo = 22;
                if (distance < 84) {
                    return .243;
                } else if (distance < 108) {
                    return .235;
                } else if (distance<132){
                    return .237;
                }else{
                    return .238;
                }
            }else{
                goalVelo = 18;
                return .23;
            }
        }
        public void raiseToAngle(double angle){
            toPosition(angle);
            }
        public void upToSpeed(double curVelo, double time){
            flyWheel.PIDConstants(kP,0,kD);
            flyWheelf.FFConstants(kV, 0, kS);
            double gain = flyWheel.calculateGain(goalVelo-curVelo, time)+flyWheelf.calculateFFFBGain(goalVelo);
            flyWheel1.setPower(gain);
            flyWheel2.setPower(gain);
            opModeObj.telemetry.addData("power",gain);
        }
        public void upToSpeed(double curVelo, double targetVelo, double time){
            flyWheel = new PIDMath(kP, 0 , kD);
            flyWheelf = new FFFBMath(kV, 0, kS);
            double gain = flyWheel.calculateGain(targetVelo-curVelo, time)+flyWheelf.calculateFFFBGain(targetVelo);
            flyWheel1.setPower(gain);
            flyWheel2.setPower(gain);
        }
        public void safetySwitch(double curVelo, double time){
            flyWheel.PIDConstants(kP,0,kD);
            flyWheelf = new FFFBMath(kV, 0, kS);
            double gain = flyWheelf.calculateFFFBGain(0) + flyWheel.calculateGain(0-curVelo, time);
            flyWheel1.setPower(gain);
            flyWheel2.setPower(gain);
        }
        public void safetySwitch(){
            flyWheel1.setPower(0);
            flyWheel2.setPower(0);
        }
        public boolean atSpeed(double curVelo){
            if((curVelo-goalVelo)<.4&&(-curVelo+goalVelo)<veloRange)
                return true;
            return false;
        }
    public boolean atSpeedP(double curVelo){
        if((curVelo-goalVelo)<veloRange&&(goalVelo-curVelo)<veloRange)
            return true;
        return false;
    }
        public boolean fire(double curVelo){

            if(atSpeedP(curVelo)) {
                pusher.setPosition(rightPusherPos);
                opModeObj.telemetry.addLine("At speed");
                return true;
            }
            else{
                opModeObj.telemetry.addLine("Not at speed");
                pusher.setPosition(leftPusherPos);
                return false;
            }
        }
    public void timedFireNN(double curVelo){
       if(shots == 4){
           done = true;
           timedCancel();
       }else if(retracted){
            if(atSpeed(curVelo)){
                timedShot();
            }
        }else if (retractTime.milliseconds() > rinterval) {
                pusher.setPosition(leftPusherPos);
                if(retractTime.milliseconds()>2*rinterval) {
                    retracted = true;
                }
            }
        opModeObj.telemetry.addData("Shots",shots);
        opModeObj.telemetry.addData("Retract time",retractTime.milliseconds());
        opModeObj.telemetry.update();
    }
    public void timedFireN(double curVelo){
        if(shots>=3){
            done = true;
        }else{
            done = false;
        }
        /*
            if (retractTime.milliseconds() > 2.0 * rinterval && atSpeed(curVelo)) {
                pusher.setPosition(rightPusherPos);
                retractTime.reset();
                shots++;
            } else if (retractTime.milliseconds() > rinterval ) {
                pusher.setPosition(leftPusherPos);
            }*/
        if(atSpeed(curVelo)&&retracted){
            timedShot();
        }
        if (retractTime.milliseconds() > rinterval) {
            pusher.setPosition(leftPusherPos);
            if(retractTime.milliseconds()>2*rinterval) {
                retracted = true;
            }
        }
            opModeObj.telemetry.addData("Retract time",retractTime.milliseconds());


            /*
        if (shots == 3) {
            done = true;
            timedCancel();
        }
            if(!done) {
                if (retracted) {
                    if (atSpeed(curVelo)) {
                        timedShot();
                    }
                } else if (retractTime.milliseconds() > rinterval) {
                    pusher.setPosition(leftPusherPos);
                    if (retractTime.milliseconds() > 2 * rinterval) {
                        retracted = true;
                    }
                }
            }

             */
    }
    public void doneReset(){
            timedCancel();
            done = false;
    }
    public void timedCancel(){

                shots = 0;
                shooterTime.reset();
            retractTime.reset();
            pusher.setPosition(leftPusherPos);
            //done = true;
            retracted = true;
    }
    public void timedShot(){
            pusher.setPosition(rightPusherPos);
            shots++;
            retractTime.reset();
            //done = false;
            retracted = false;

    }

    public void liftUp(){
            lift.setPosition(highLift);
    }
    public void liftDown(){
            lift.setPosition(lowerLift);
    }
    public void toPosition(double pos){
            flap.setPosition(Range.clip(pos,lowerFlap,highFlap));
    }

    }


