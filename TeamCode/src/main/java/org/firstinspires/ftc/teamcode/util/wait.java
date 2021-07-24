package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class wait {
    public ElapsedTime waitClock;
    double startTime = 999999999;
    double length;
    boolean repeat;
    public boolean init;
    public wait(ElapsedTime waitClock, double length){
        this.waitClock = waitClock;
        this.length = length;
        startTime = this.waitClock.seconds();
        this.repeat = true;
    }
    public wait(double length, boolean repeat){
        waitClock = new ElapsedTime();
        waitClock.reset();
        this.length = length;
        this.repeat = repeat;
        init = false;
    }
    public void init(){
        if(init == false||repeat) {
            waitClock.reset();
            init = true;
        }
    }
    public void deinit(){
        if(init == true){
            init = false;
        }
    }
    public boolean timeUp(){
        if(waitClock.seconds()>=length){
            return true;
        }
        else{
            return false;
        }
    }
}
