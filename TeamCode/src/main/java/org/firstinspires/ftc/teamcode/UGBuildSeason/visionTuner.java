package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.geometry.Point;
import org.firstinspires.ftc.teamcode.subSystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;
import org.firstinspires.ftc.teamcode.subSystems.hood;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.reader;
import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator;
import org.firstinspires.ftc.teamcode.subSystems.vision;
import org.firstinspires.ftc.teamcode.subSystems.wobble;
import org.firstinspires.ftc.teamcode.util.wait;
import org.firstinspires.ftc.teamcode.utilnonrr.FieldCoordinatesB;

@Autonomous
@Config
public class visionTuner extends LinearOpMode {


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


        int stack = camera.height();
        telemetry.addData("Stack height",stack);
        telemetry.addLine("Init Successful");
        telemetry.update();
        waitForStart();
        wait vision = new wait(runtime,.5);
        while(!vision.timeUp()&&!isStopRequested()&&opModeIsActive()){
            stack = camera.height();
        }
        telemetry.addData("Stack height",stack);
        telemetry.update();
        runtime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            stack = camera.height();
            telemetry.addData("Stack height",stack);
            telemetry.update();
    }
    }

}

