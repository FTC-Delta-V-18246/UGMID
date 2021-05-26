package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.geometry.Point;

public class movement {
    public static double kP = 0, kD = 0, kF = 0;
    public movement(LinearOpMode opMode, hardwareGenerator gen, double kP, double kD, double kF){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
    }
    public double robotAngleToTarget(Point target, Pose2d curPos){
        double tgtAngle = 3* Math.PI/2.0 - Math.atan2(target.x-curPos.getX(),target.y-curPos.getY());
        //opModeObj.telemetry.addData("Target Angle", tgtAngle);
        return tgtAngle;
    }
    public double turnToAbsolute(Point target, Pose2d curPos){
        double tgtAngle = robotAngleToTarget(target, curPos);
        if(Math.abs(tgtAngle-curPos.getHeading())< Math.abs(-2* Math.PI+tgtAngle-curPos.getHeading())){
            return tgtAngle-curPos.getHeading();
        }
        else{
            return -2* Math.PI+tgtAngle-curPos.getHeading();
        }
        //return tgtAngle - curPos.getHeading();
    }
    public double turnToAbsolute(double tgtAngle, Pose2d curPos){
        if(Math.abs(tgtAngle-curPos.getHeading())< Math.abs(-2* Math.PI+tgtAngle-curPos.getHeading())){
            return tgtAngle-curPos.getHeading();
        }
        else{
            return -2* Math.PI+tgtAngle-curPos.getHeading();
        }
        //return tgtAngle - curPos.getHeading();
    }
}
