package org.firstinspires.ftc.teamcode.utilnonrr;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static java.lang.Math.PI;

@Disabled
public class MathFunctions {
    public static double AngleWrap(double angle){
        while(angle<-PI){
            angle+=2*PI;
        }
        while(angle>PI){
            angle-=2*PI;
        }
        return angle;
    }
}
