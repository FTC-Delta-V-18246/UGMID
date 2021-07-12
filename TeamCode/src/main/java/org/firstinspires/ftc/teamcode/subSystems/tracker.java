package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;

/*
CODES:
-1 : rings are still entering the canister
-2 : rings are in motion within the canister
 */
@Config
public class tracker {
    public static double vTol = .1;
    //public static double xTol = .01;
    public static double minDis = 25;
    public static double ringIII = 29;
    public static double ringII = 33.7;
    public static double ringI = 36;
    public static double ring0 = 37;
    hardwareGenerator gen;
    public double curX, prevX = 0, curVelo;
    private double time, sum;
    ElapsedTime timer;
    public double[] rH = {ring0, ringI, ringII, ringIII, minDis};
    public tracker(hardwareGenerator gen, ElapsedTime timer){
        this.gen = gen;

        this.timer = timer;
    }
    public int counter(double curX){
        curVelo = (curX-prevX);
        prevX = curX;
        if(Math.abs(curVelo)>vTol) return -2;
        for(int i = 0; i<4;i++){
            if(curX>(rH[i]+rH[i+1])/2.0) return i;
        }
        return -1;

    }
    public double tuner(double curX){
        sum+=curX;
        curVelo = (curX-prevX);
        prevX = curX;
        time++;
        return (double)sum/time;
    }
    public void tunerReset(){
       time = 0;
       sum = 0;

    }

}

