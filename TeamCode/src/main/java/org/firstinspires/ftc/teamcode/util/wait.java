package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class wait {
    ElapsedTime waitClock;
    double startTime;
    double length;
    public wait(ElapsedTime waitClock, double length){
        this.waitClock = waitClock;
        this.length = length;
        startTime = this.waitClock.seconds();
    }
    public boolean timeUp(){
        if(waitClock.seconds()-startTime>=length){
            return true;
        }
        else{
            return false;
        }
    }
}
