package org.firstinspires.ftc.teamcode.utilnonrr;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class PIDMath {
    private double P, I, D, F;
    double pastError;
    double totalError;
    double prevTime;
    public PIDMath(double kP, double kI, double kD){
     P = kP;
     I = kI;
     D = kD;
     F = 0;
    }
    public PIDMath(double kP, double kI, double kD, double kF){
        P = kP;
        I = kI;
        D = kD;
        F = kF;
    }
    public double calculateGain(double error, double nowTime){
        totalError += error;
        double gain = error * P + totalError * I + D * (error-pastError)/(nowTime-prevTime)+ Math.signum(error)*F;
        prevTime = nowTime;
        pastError = error;
        return gain;

    }
    public void PIDConstants(double kP, double kI, double kD, double kF){
        P = kP;
        I = kI;
        D = kD;
        F = kF;
    }
    public void PIDConstants(double kP, double kI, double kD){
        P = kP;
        I = kI;
        D = kD;
    }
    public void resetD(){
        prevTime = 0;
        pastError = 0;
    }
}
