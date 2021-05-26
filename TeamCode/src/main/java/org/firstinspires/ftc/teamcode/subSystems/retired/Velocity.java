package org.firstinspires.ftc.teamcode.subSystems.retired;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class Velocity {
    private double prevTime;
    private double prevCount;
    private DcMotorEx targetMotor;
    private ElapsedTime clock;
    double curTime;
    private double velocity;
    ArrayList<Double> velocities;
    public Velocity(DcMotorEx targetMotor, ElapsedTime clock){
        this.targetMotor = targetMotor;
        this.clock = clock;
        prevCount = targetMotor.getCurrentPosition();
        prevTime = clock.seconds();
        velocities = new ArrayList<Double>();
    }
    public double calculateVelocity(double targetPos){
        curTime = clock.seconds();
        velocity = ((targetPos-prevCount)/28)/(curTime-prevTime);
        prevCount = targetPos;
        prevTime = curTime;
        //velocities.
        return velocity;
    }
}
