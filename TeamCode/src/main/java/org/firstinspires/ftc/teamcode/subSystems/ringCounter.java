package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;

/*
CODES:
-1 : rings are still entering the canister
-2 : rings are in motion within the canister
 */
public class ringCounter {
    public static double vTol = .1;
    //public static double xTol = .01;
    public static double minDis = 20;
    public static double ringIII = 25.4;
    public static double ringII = 50.8;
    public static double ringI = 76.2;
    public static double ring0 = 101.6;
    hardwareGenerator gen;
    SensorMRRangeSensor dSensor;
    double curX, prevX = 0, curVelo, prevT,curT = 0;
    ElapsedTime timer;
    public double[] rH = {minDis, ringIII,ringII,ringI,ring0};
    public ringCounter(hardwareGenerator gen, ElapsedTime timer){
        this.gen = gen;
        //dSensor = gen.dSensor;
        this.timer = timer;
    }
    public int counter(){
       //curX = dSensor.cmOptical();
       curT = timer.seconds();
       curVelo = (curX-prevX)/(curT-prevT);
        prevX = curX;
        prevT = curT;
       if(curX<minDis){
           return -1;
       }else if(Math.abs(curVelo)>Math.abs(vTol)){
            return -2;
       }else{
          return withinTol(curX);
       }

    }
    public int withinTol(double curX){
        for(int i = 1; i<4; i++){
            if(curX<(rH[i]+rH[i-1])/2.0){
                return -1;
            }
            if(curX>(rH[i]+rH[i-1])/2.0&&curX<(rH[i]+rH[i+1])/2.0){
                return 4-i;
            }
        }
        return 0;
    }
}
