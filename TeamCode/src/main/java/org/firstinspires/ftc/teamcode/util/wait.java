package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class wait {
    ElapsedTime waitClock;
    double startTime = 999999999;
    double length;
    boolean repeat;
    boolean init;
    public wait(ElapsedTime waitClock, double length){
        this.waitClock = waitClock;
        this.length = length;
        startTime = this.waitClock.seconds();
        this.repeat = true;
    }
    public wait(ElapsedTime waitClock, double length, boolean repeat){
        this.waitClock = waitClock;
        this.length = length;
        this.repeat = false;
        init = false;
    }
    public void init(){
        if(init == false) {
            waitClock.reset();
            startTime = this.waitClock.seconds();
            init = true;
        }
    }
    public void deinit(){
        if(init == true){
            init = false;
        }
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
