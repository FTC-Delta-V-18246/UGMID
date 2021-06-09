package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.UGBuildSeason.teleB;
import org.firstinspires.ftc.teamcode.geometry.Point;
import org.firstinspires.ftc.teamcode.utilnonrr.FFFBMath;
import org.firstinspires.ftc.teamcode.utilnonrr.PIDMath;

@Config
public class hood {
        LinearOpMode opModeObj;
        public LinearServo angler;
        public static double goalVelo = 18;
        private DcMotorEx flyWheel1 = null;
        private DcMotorEx flyWheel2 = null;
        private Servo pusher;
        private DigitalChannel limitSwitch;
        public static double leftPusherPos = .215;
        public static double rightPusherPos = .31; //.3
        public static double kP = 0,kD = 0,fireSpeed = 0;
        public static double kV = 0, kS = 0;
        public static double feedAngle = .323; //
        public static double interval = 150;
        public static double rinterval = 70; // minimum of 45 (realistically 55)
        public static double veloRange = 1.5; //max of 3, probably could be increased if we increased rinterval
        public static double shooterHeight = .15;
        public double shots = 0;
        public boolean save = false;
        boolean retracted = true;
        public boolean done = false;
        ElapsedTime shooterTime = new ElapsedTime();
        ElapsedTime retractTime = new ElapsedTime();
        private PIDMath flyWheel;
        private FFFBMath flyWheelf;
        public hood(LinearOpMode opMode, hardwareGenerator gen, LinearServo angler, double fireSpeed, double kP, double kD, double kS, double kV){
            opModeObj = opMode;
            this.angler = angler;
            this.kP = kP;
            this.kD = kD;
            this.kV = kV;
            this.kS = kS;
            this.fireSpeed = fireSpeed;
            flyWheel = new PIDMath(kP, 0 , kD);
            flyWheelf = new FFFBMath(kV, 0, kS);
            goalVelo = fireSpeed;
            flyWheel1 = gen.flyWheelM;
            flyWheel2 = gen.flyWheelM1;
            limitSwitch = gen.shooterLimitSwitch;
            pusher = gen.pusherServo;
            shooterTime.reset();
        }
        public double calculateTargetShooterAngle(Point target, Pose2d robot, boolean power){
            double distance = Math.sqrt(Math.pow(target.x-robot.getX(),2)+ Math.pow(target.y-robot.getY(),2));
            opModeObj.telemetry.addData("Distance",distance);
            if(!power){
                return 5.5357014*Math.pow(10,-5)*Math.pow(distance,2)-.0106856205*distance+.7315347531;//shooterHeight;//
            }else{
                return shooterHeight;
            }
        }
        public void raiseToAngle(double angle){
            angler.toPosition(angle);
            }
        public void upToSpeed(double curVelo, double time){
            flyWheel.PIDConstants(kP,0,kD);
            flyWheelf.FFConstants(kV, 0, kS);
            goalVelo = fireSpeed;
            double gain = flyWheel.calculateGain(goalVelo-curVelo, time)+flyWheelf.calculateFFFBGain(goalVelo);
            flyWheel1.setPower(gain);
            flyWheel2.setPower(gain);
            opModeObj.telemetry.addData("power",gain);
        }
        public void upToSpeed(double curVelo, double targetVelo, double time){
            flyWheel = new PIDMath(kP, 0 , kD);
            flyWheelf = new FFFBMath(kV, 0, kS);
            goalVelo = fireSpeed;
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
            if((curVelo-goalVelo)<.4&&(goalVelo-curVelo)<veloRange)
                return true;
            return false;
        }
    public boolean atSpeedP(double curVelo){
        if((curVelo-goalVelo)<veloRange&&(goalVelo-curVelo)<veloRange)
            return true;
        return false;
    }
        public void feed(){
            angler.toPosition(feedAngle);
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
        public void timedFire(double curVelo){
            if(save){
                if(atSpeed(curVelo)){
                    save = false;
                }
                pusher.setPosition(leftPusherPos);
            }else {
                if (shots == 2 && shooterTime.milliseconds() >= 2 * interval&&retracted) {
                    timedShot();
                } else if (shots == 1 && shooterTime.milliseconds() >= interval&&retracted) {
                    timedShot();
                } else if (atSpeed(curVelo) && shots == 0) {
                    timedShot();
                    shooterTime.reset();
                } else if (shots != 0) {
                    if (retractTime.milliseconds() > rinterval) {
                        pusher.setPosition(leftPusherPos);
                        retracted = true;
                    }
                }
                if (shots == 3) {
                    timedCancel();
                }
            }
            opModeObj.telemetry.addData("Shots",shots);
            opModeObj.telemetry.addData("Shooter time",shooterTime.milliseconds());
            opModeObj.telemetry.addData("Retract time",retractTime.milliseconds());
        }
    public void timedFireN(double curVelo){
        if(retracted){
            if (shots == 4) {
                timedCancel();
            } else if(atSpeed(curVelo)){
                timedShot();
            }
        }else{
            if (retractTime.milliseconds() > rinterval) {
                if(shots == 3){
                    done = true;
                    teleB.shooterState = teleB.RobotState.INDEXING;
                    timedCancel();
                }
                pusher.setPosition(leftPusherPos);
                if(retractTime.milliseconds()>2*rinterval) {
                    retracted = true;
                } //intellij
            }
        }
        opModeObj.telemetry.addData("Shots",shots);
        opModeObj.telemetry.addData("Retract time",retractTime.milliseconds());
    }
        public void timedCancel(){

                shots = 0;
                shooterTime.reset();
            retractTime.reset();
            pusher.setPosition(leftPusherPos);
            retracted = true;
        }
        public void timedShot(){
            pusher.setPosition(rightPusherPos);
            shots++;
            retractTime.reset();
            done = false;
            retracted = false;

        }

    }


