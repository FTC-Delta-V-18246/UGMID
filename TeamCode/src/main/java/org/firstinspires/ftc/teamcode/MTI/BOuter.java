package org.firstinspires.ftc.teamcode.MTI;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import org.apache.commons.math3.stat.descriptive.moment.VectorialCovariance;
import org.firstinspires.ftc.teamcode.UGBuildSeason.Bbasic;
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
import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator.*;
import org.firstinspires.ftc.teamcode.subSystems.vision;
import org.firstinspires.ftc.teamcode.subSystems.wobble;
import org.firstinspires.ftc.teamcode.util.wait;
import org.firstinspires.ftc.teamcode.utilnonrr.FieldCoordinatesB;

import java.util.Arrays;
import java.util.Vector;

@Autonomous
@Config
public class BOuter extends LinearOpMode{
    public SampleMecanumDrive driver;
    public reader hardReader;
    public hood shooter;
    public intake roller;
    public vision camera;
    public wobble hammer;

    private ElapsedTime runtime = new ElapsedTime();

    private enum State{
        preLoad,
        stackB,
        stackC,
        stackCI,
        wobbleI,
        park
    }
    Trajectory backUp, forward;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void runOpMode() throws InterruptedException {


        hardwareGenerator gen = new hardwareGenerator(this);
        subsystemGenerator subs = new subsystemGenerator(this, gen, runtime);
        this.driver = subs.driver;
        this.hardReader = subs.hardReader;
        this.shooter = subs.shooter;
        this.roller = subs.roller;
        this.camera = subs.camera;
        this.hammer = subs.hammer;

        FieldCoordinatesB field = new FieldCoordinatesB();

        Pose2d startPose = new Pose2d(-64,49, 0);
        driver.setPoseEstimate(startPose);
        hardReader.curPose = startPose;


        Trajectory preL = driver.trajectoryBuilder(startPose)
                .lineToConstantHeading(field.PRLC.vec())
                .addDisplacementMarker(() ->
                        roller.upToSpeed(-.8))
                .addDisplacementMarker(() ->
                        roller.fallOut())
                .build();


        Trajectory wobbleA = driver.trajectoryBuilder(preL.end())
                .lineToSplineHeading(field.WAI)
                .build();
        Trajectory parkA = driver.trajectoryBuilder(wobbleA.end())
                //.splineToConstantHeading(new Vector2d(-10,30),0)
                .splineToLinearHeading(field.PAM,0)
                .build();


        Trajectory stackBI = driver.trajectoryBuilder(preL.end())
                .lineToSplineHeading(new Pose2d(-24,field.align,0))
                .build();
        Trajectory wobbleB = driver.trajectoryBuilder(stackBI.end())
                .splineToSplineHeading(field.WBI,Math.PI)
                .build();
        Trajectory parkB = driver.trajectoryBuilder(wobbleB.end())
                .lineToSplineHeading(field.PAR)
                .build();

        wait takeIn = new wait(3,true);

        Trajectory KO = driver.trajectoryBuilder(preL.end())
                .addDisplacementMarker(() ->
                        roller.upToSpeed(.8))
                .addDisplacementMarker(() ->
                        roller.fallOut())
                .lineToSplineHeading(new Pose2d(-40,field.align,0),new MinVelocityConstraint(
                Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(.2*DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                /*.addDisplacementMarker(() ->
                        /*driver.followTrajectoryAsync(backUp))*/
                .build();
        backUp = driver.trajectoryBuilder(KO.end())
                .lineToSplineHeading(new Pose2d(-20,field.align,0))
                .addDisplacementMarker(() ->
                        driver.followTrajectoryAsync(forward)
                        )
                .build();
        forward = driver.trajectoryBuilder(backUp.end())
                .addDisplacementMarker(() ->
                        roller.upToSpeed(.7)
                )
                .lineToSplineHeading(new Pose2d(-37,field.align,0),new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(.2*DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() ->
                        takeIn.init())
                .build();
        Trajectory last = driver.trajectoryBuilder(forward.end())
                .lineToSplineHeading(new Pose2d(-34,field.align,0),new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(1*DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory wobbleC = driver.trajectoryBuilder(last.end())
                .lineToSplineHeading(field.WCI)
                .build();

        Trajectory parkC = driver.trajectoryBuilder(wobbleC.end())
                .lineToSplineHeading(field.PAR)
                .build();


//intake at 10.5 inches from the stack - measured from the gotube of the transfer to the center of the seam the four stack is on.

        hammer.preload();
        waitForStart();

        int stack = 4;
        /*while (!isStopRequested() && opModeIsActive()&&!Thread.currentThread().isInterrupted()) {
            stack = camera.height();
            telemetry.addData("Stack height", stack);
            telemetry.update();
        }

         */

        roller.fallOut();
        shooter.doneReset();
        shooter.timedCancel();
        runtime.reset();

        boolean noRepeat = false;
        boolean aligned = false;
        boolean shooterDone = false;
        wait lift = new wait(2, true);
        wait liftD = new wait(.2,true);
        wait preload = new wait(.5, false);
        wait wobblePause = new wait(18,true);
        State currentState = State.preLoad;
        driver.followTrajectoryAsync(preL);

        while (!isStopRequested() && opModeIsActive()) {
            hardReader.autonRead();
            Pose2d curPose = hardReader.curPose;
            PoseStorage.currentPose = curPose;

            switch(currentState){
                case preLoad:
                    //roller.tuckIn();
                    preload.init();
                    shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose,false));

                    if(!driver.isBusy()&&lift.timeUp()&&preload.timeUp()&&lift.init){
                        if(!aligned){
                            driver.turnAsync(field.HM);
                            aligned = true;
                        }else {
                            shooter.timedFireN(hardReader.shooterV);
                            if (shooter.done) {
                                shooter.timedCancel();
                                shooter.liftDown();
                                aligned = false;
                                switch (stack) {
                                    case 0:
                                        // drive to drop wobble
                                        driver.followTrajectoryAsync(wobbleA);
                                        wobblePause.init();
                                        hammer.lowLift();
                                        currentState = State.wobbleI;
                                        break;
                                    case 1:
                                        // drive to intake stack
                                        shooter.liftDown();
                                        driver.followTrajectoryAsync(stackBI);
                                        roller.upToSpeed();
                                        takeIn.init();
                                        currentState = State.stackB;
                                        break;
                                    case 4:
                                        // drive to intake stack
                                        driver.followTrajectoryAsync(KO);
                                        currentState = State.stackC;
                                        break;
                                }
                                shooter.doneReset();
                            }
                        }
                    }
                    if(preload.timeUp()){
                        hammer.partialLift();
                        if(!noRepeat) {
                            lift.init();
                            shooter.liftUp();
                            noRepeat = true;
                        }
                    }
                    break;
                case stackB:
                    if(!driver.isBusy()){
                        if(takeIn.timeUp()) {
                            roller.upToSpeed(0);
                            shooter.liftUp();
                            if (lift.timeUp()) {
                                shooter.timedFireN(hardReader.shooterV);
                                if (shooter.done) {
                                    shooter.doneReset();
                                    shooter.liftDown();
                                    driver.followTrajectoryAsync(wobbleB);
                                    hammer.lowLift();
                                    currentState = State.wobbleI;
                                }
                            }
                        }else{
                            lift.init();
                        }
                    }else{
                        lift.init();
                        takeIn.init();
                    }
                    break;

                case stackC:
                    if(!driver.isBusy()){
                        if(takeIn.timeUp()) {
                            roller.upToSpeed(0);
                            shooter.liftUp();
                            if (lift.timeUp()) {
                                if(shooter.shots>=3){
                                    gen.pusherServo.setPosition(hood.leftPusherPos);
                                    shooterDone = true;
                                }else{
                                    shooter.timedFireN(hardReader.shooterV);
                                }
                                if (shooterDone) {
                                    shooter.liftDown();
                                    if (liftD.timeUp()) {
                                        driver.followTrajectoryAsync(last);
                                        //takeIn.init();
                                        lift.init();
                                        shooterDone = false;
                                        roller.upToSpeed(.7);
                                        shooter.doneReset();
                                        currentState = State.stackCI;
                                    }
                                } else {
                                    shooter.timedFireN(hardReader.shooterV);
                                    liftD.init();
                                }
                            }
                        }
                        else{
                            lift.init();
                        }
                    }
                    else{
                        takeIn.init();
                        lift.init();
                    }
                    break;
                case stackCI:
                    if(!driver.isBusy()){
                        if(takeIn.timeUp()) {
                            roller.upToSpeed(0);
                            shooter.liftUp();
                            if (lift.timeUp()) {
                                if(shooter.shots>=3){
                                    gen.pusherServo.setPosition(hood.leftPusherPos);
                                    shooterDone = true;
                                }else{
                                    shooter.timedFireN(hardReader.shooterV);
                                }
                                if (shooterDone) {
                                    shooter.liftDown();
                                    if (liftD.timeUp()) {
                                        driver.followTrajectoryAsync(wobbleC);
                                        //takeIn.init();
                                        lift.init();
                                        currentState = State.wobbleI;
                                    }
                                } else {
                                    shooter.timedFireN(hardReader.shooterV);
                                    liftD.init();
                                }
                            }
                        }
                        else{
                            lift.init();
                        }
                    }
                    else{
                        takeIn.init();
                        lift.init();
                    }
                    break;
                case wobbleI:
                    roller.tuckIn();
                    if(!driver.isBusy()) {
                        hammer.release();
                    }
                    if(!driver.isBusy()){
                        switch(stack) {
                            case 0:
                                if(wobblePause.timeUp()) {
                                    driver.followTrajectoryAsync(parkA);
                                }
                                break;
                            case 1:
                                driver.followTrajectoryAsync(parkB);
                                currentState = State.park;
                                break;
                            case 4:
                                driver.followTrajectoryAsync(parkC);
                                currentState = State.park;
                                break;
                        }
                    }
                    break;
                case park:
                    shooter.liftDown();
                    hammer.partialLift();
                    shooter.toPosition(shooter.levelFlap);
                    break;

            }

            telemetry.addData("State",currentState);
            telemetry.addData("Intake timer",takeIn.timeUp());
            telemetry.addData("Intake clock", takeIn.waitClock.seconds());
            telemetry.addData("Driver busy?", driver.isBusy());
            telemetry.addData("Shooter done?",shooterDone);
            telemetry.update();
            driver.update();


            if(currentState == State.preLoad || currentState == State.stackB || currentState == State.stackC || currentState == State.stackCI){
                shooter.upToSpeed(hardReader.shooterV, runtime.seconds());
            }
            else{
                shooter.safetySwitch();
            }

        }

    }
}
