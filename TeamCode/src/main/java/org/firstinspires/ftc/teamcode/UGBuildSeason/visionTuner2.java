package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;
import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator;
import org.firstinspires.ftc.teamcode.subSystems.vision2;
import org.firstinspires.ftc.teamcode.util.wait;

@Autonomous
@Config
public class visionTuner2 extends LinearOpMode {


    public vision2 camera;
    enum Rings{
        ZERO,
        ONE,
        FOUR,
    }
ElapsedTime runtime = new ElapsedTime();
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void runOpMode() throws InterruptedException {


        hardwareGenerator gen = new hardwareGenerator(this);
        subsystemGenerator subs = new subsystemGenerator(this, gen, runtime);
        vision2 camera2 = new vision2(this);


        int stack = camera2.height();
        telemetry.addData("Stack height",stack);
        telemetry.addLine("Init Successful");
        telemetry.update();
        waitForStart();
        wait vision = new wait(runtime,.5);
        while(!vision.timeUp()&&!isStopRequested()&&opModeIsActive()){
            stack = camera2.height();
        }
        telemetry.addData("Stack height",stack);
        telemetry.update();
        runtime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            stack = camera2.height();
            telemetry.addData("Stack height",stack);
            telemetry.update();
    }
    }

}

