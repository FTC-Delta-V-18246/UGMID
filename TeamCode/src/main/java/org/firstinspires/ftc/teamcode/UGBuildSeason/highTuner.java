package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subSystems.UGAngleHighGoalPipeline;
import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;
import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator;
import org.firstinspires.ftc.teamcode.subSystems.vision;
import org.firstinspires.ftc.teamcode.util.wait;

@TeleOp
@Config
public class highTuner extends LinearOpMode {


    public vision camera;
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

        this.camera = subs.camera;
        camera.highGoal();
        telemetry.addLine("Init Successful");
        telemetry.update();
        waitForStart();

        /*wait vision = new wait(runtime,.5);
        while(!vision.timeUp()&&!isStopRequested()&&opModeIsActive()){
            stack = camera.height();
        }
        telemetry.addData("Stack height",stack);
        telemetry.update();
        */
        runtime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            subs.hardReader.teleRead();
            telemetry.addData("Angle BLUE", camera.goalline.calculateYaw(UGAngleHighGoalPipeline.Target.BLUE));
            telemetry.addData("Angle RED", camera.goalline.calculateYaw(UGAngleHighGoalPipeline.Target.RED));
            telemetry.addData("RED?",camera.goalline.isRedVisible());
            telemetry.addData("BLUE?",camera.goalline.isBlueVisible());
            subs.shooter.toPosition(.26);
            if(camera.goalline.isBlueVisible()) {
                subs.driver.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, camera.goalline.angleAlign(UGAngleHighGoalPipeline.Target.BLUE),Math.toDegrees(subs.hardReader.curPose.getHeading())+90);
                telemetry.addData("Which way", camera.goalline.angleAlign(UGAngleHighGoalPipeline.Target.BLUE));
            }else{
                subs.driver.driveFieldCentric(0, 0,0, 0);
            }



            telemetry.update();
    }
    }

}

