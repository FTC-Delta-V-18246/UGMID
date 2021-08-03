package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class wobble {
    public Servo wobbleClaw, wobblePivot;
    public static double wobbleGrab = .6, wobbleRelease = 1, wobbleLoad = .4, wobbleLift = .5, wobbleDown = 0.05, wobblePartialLift = .2, lowLift = .1;
    public wobble(LinearOpMode opMode, hardwareGenerator gen){
        wobbleClaw = gen.wobbleClaw;
        wobblePivot = gen.wobblePivot;
    }
    public void preload(){
        wobbleClaw.setPosition(.64);
        wobblePivot.setPosition(wobbleLoad);
    }
    public void grab(){
        wobbleClaw.setPosition(wobbleGrab);
    }
    public void lift(){
        wobblePivot.setPosition(wobbleLift);
    }
    public void down(){
        wobblePivot.setPosition(wobbleDown);
    }
    public void release(){
        wobbleClaw.setPosition(wobbleRelease);
    }
    public void partialLift(){
        wobblePivot.setPosition(wobblePartialLift);
    }
    public void lowLift(){
        wobblePivot.setPosition(lowLift);
    }
}
