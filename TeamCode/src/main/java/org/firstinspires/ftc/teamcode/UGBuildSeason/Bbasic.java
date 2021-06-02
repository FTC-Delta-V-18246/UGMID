package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subSystems.*;
import org.firstinspires.ftc.teamcode.util.wait;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

import java.util.Arrays;

@Autonomous
@Config
public class Bbasic extends LinearOpMode {

    public SampleMecanumDrive driver;
    public reader hardReader;
    public hood shooter;
    public intake roller;
    public vision camera;
    public wobble hammer;

    private enum State {
        dTWD,
        WD,
        dTS,
        S,
        dTIS,
        IS,
        dTISI,
        ISI,
        dTPA,
        idle,
    }
    private enum PowerShot{
        L,
        M,
        R,
    }

    private ElapsedTime runtime = new ElapsedTime();

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void runOpMode() throws InterruptedException {

        wait timer;
        wait timer2;
        boolean timerDone = false;
        boolean shotFired = false;
        hardwareGenerator gen = new hardwareGenerator(this);
        subsystemGenerator subs = new subsystemGenerator(this, gen, runtime);
        this.shooter = subs.shooter;
        this.roller = subs.roller;
        this.driver = subs.driver;
        this.camera = subs.camera;
        this.hammer = subs.hammer;
        this.hardReader = subs.hardReader;

        FieldCoordinatesB field = new FieldCoordinatesB();
        Pose2d startPose = new Pose2d(-64,49, 0);
        driver.setPoseEstimate(startPose);
        hardReader.curPose = startPose;

        Trajectory wobbleA = driver.trajectoryBuilder(startPose)
                .lineToSplineHeading(field.WAI)
                .build();
        Trajectory preLA = driver.trajectoryBuilder(wobbleA.end())
                .lineToConstantHeading(field.PRLC.vec())
                .build();


        Trajectory wobbleB = driver.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-12, 50),0)
                .splineToSplineHeading(field.WBI,0)
                .build();
        Trajectory preLB = driver.trajectoryBuilder(wobbleB.end())
                .lineToConstantHeading(field.PRLC.vec())
                .build();

        Trajectory wobbleC = driver.trajectoryBuilder(startPose)
                .lineToSplineHeading(field.WCI)
                .build();
        Trajectory preLC = driver.trajectoryBuilder(wobbleC.end())
                .lineToConstantHeading(field.PRLC.vec())
                .build();

        Trajectory intakeI = driver.trajectoryBuilder(preLC.end())
                .lineToConstantHeading(new Vector2d(-22, 36), new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(2, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory intakeII = driver.trajectoryBuilder(intakeI.end())
                .lineToConstantHeading(new Vector2d(-30, 36), new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(2, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory parkA = driver.trajectoryBuilder(preLA.end())
                //.splineToConstantHeading(new Vector2d(-10,30),0)
                .splineToConstantHeading(field.PAL.vec(),0)
                .build();
        Trajectory parkB = driver.trajectoryBuilder(intakeI.end())
                .lineToConstantHeading(field.PAR.vec())
                .build();
        Trajectory parkC = driver.trajectoryBuilder(intakeII.end())
                .lineToConstantHeading(field.PAR.vec())
                .build();


        hammer.grab();
        hammer.lift();
        waitForStart();
        int stack = 0;
        wait vision = new wait(runtime, .5);
        shooter.timedCancel();
        while (!vision.timeUp() && !isStopRequested() && opModeIsActive()) {
            stack = camera.height();
        }
        telemetry.addData("Stack height", stack);
        telemetry.update();
        roller.fallOut();
        runtime.reset();
        wait wobbleDrop = new wait(runtime,.2), wobbleGrab= new wait(runtime,.2);;
        State currentState = State.dTWD;

        switch (stack) {
            case 0:
                driver.followTrajectoryAsync(wobbleA);
                break;
            case 1:
                driver.followTrajectoryAsync(wobbleB);
                break;
            case 4:
                driver.followTrajectoryAsync(wobbleC);
                break;
        }
        while (!isStopRequested() && opModeIsActive()) {
            hardReader.autonRead();
            Pose2d curPose = hardReader.curPose;
            PoseStorage.currentPose = curPose;
            switch (currentState) {
                case dTWD:
                    wobbleDrop = new wait(runtime,.8);
                    if(!driver.isBusy()){
                        hammer.lowLift();
                        currentState = State.WD;
                    }
                    break;
                case WD:
                    if(wobbleDrop.timeUp()){
                        hammer.release();
                        switch(stack){
                            case 0:
                                //drive to shoot
                                driver.followTrajectoryAsync(preLA);
                                break;
                            case 1:
                                //drive to shoot
                                driver.followTrajectoryAsync(preLB);
                                break;
                            case 4:
                                //drive to shoot
                                driver.followTrajectoryAsync(preLC);
                        }
                        currentState = State.dTS;
                    }
                    break;
                case dTS:
                    if(!driver.isBusy()){
                        currentState = State.S;
                    }
                    break;
                case S:
                    shooter.timedFireN(hardReader.shooterV); //turn on shooter during S
                    if(shooter.done){
                        switch(stack){
                            case 0:
                                //park
                                driver.followTrajectoryAsync(parkA);
                                currentState = State.dTPA;
                                break;
                            case 1:
                            case 4:
                                //drive to intake the stack, slow speed
                                driver.followTrajectoryAsync(intakeI);
                                roller.upToSpeed();
                                currentState = State.dTIS;
                                break;
                        }
                    }
                    break;
                case dTIS:
                    if(!driver.isBusy()){
                        roller.upToSpeed(0);
                        currentState = State.IS;
                    }
                    break;
                case IS:
                    shooter.timedFireN(hardReader.shooterV); //turn on shooter during IS
                    if(shooter.shots>2){
                        switch(stack){
                            case 1:
                                //park
                                driver.followTrajectoryAsync(parkB);
                                shooter.timedCancel();
                                currentState = State.dTPA;
                                break;
                            case 4:
                                //drive to intake the stack, slow speed
                                shooter.timedCancel();
                                driver.followTrajectoryAsync(intakeII);
                                roller.upToSpeed();
                                currentState = State.dTISI;
                                break;
                        }
                    }
                    break;
                case dTISI:
                    if(!driver.isBusy()){
                        roller.upToSpeed(0);
                        currentState = State.ISI;
                    }
                    break;
                case ISI:
                    shooter.timedFireN(hardReader.shooterV); //turn on shooter during ISI
                    if(shooter.done){
                        //park
                        driver.followTrajectoryAsync(parkC);
                        currentState = State.dTPA;
                    }
                    break;
                case dTPA:
                    if(!driver.isBusy()){
                        currentState = State.idle;
                    }
                    break;
                case idle:
                    hammer.grab();
                    hammer.lift();
                    roller.fallOut();
                    break;
            }
            if (currentState == State.S || currentState == State.IS|| currentState == State.ISI) {
                shooter.upToSpeed(hardReader.shooterV, runtime.seconds());
            } else {
                gen.flyWheelM.setPower(0);
                gen.flyWheelM1.setPower(0);
            }
            driver.update();
            telemetry.addData("At velo?", shooter.atSpeed(hardReader.shooterV));
            telemetry.addData("Velo", hardReader.shooterV);
            telemetry.addData("State",currentState);
            telemetry.update();
        }


    }


}


