package org.firstinspires.ftc.teamcode.subSystems;

import org.firstinspires.ftc.teamcode.util.wait;

public class stockTimer {
    public wait lift, liftD, preload, shotTimer, retract, wobblePause, wobblePickup;
    public stockTimer(){
        lift = new wait(1, true);
        liftD = new wait(.2,false);
        //wait takeIn = new wait(runtime,3,true);
        preload = new wait(.5, false);
        shotTimer = new wait(.7,true);
        retract = new wait(.4,false);
        wobblePause = new wait(18,true);
        wobblePickup = new wait(1.2,false);

    }
    public void start(){
        lift.init();
        liftD.init();
        preload.init();
        shotTimer.init();
        retract.init();
        wobblePause.init();
        wobblePickup.init();

        lift.deinit();
        liftD.deinit();
        preload.deinit();
        shotTimer.deinit();
        retract.deinit();
        wobblePause.deinit();
        wobblePickup.deinit();

    }
}
