package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

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
import org.firstinspires.ftc.teamcode.MTI.FieldCoordinatesR;

import java.util.Arrays;

@Autonomous
@Config
public class Rbasic extends LinearOpMode {

    public SampleMecanumDrive driver;
    public reader hardReader;
    public hood shooter;
    public intake roller;
    public vision camera;
    public wobble hammer;

    private enum State {
        dTWD,
        WD,
        dTTTS,
        dTTS,
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

        FieldCoordinatesR field = new FieldCoordinatesR();
        Pose2d startPose = new Pose2d(-64,-49, 0);
        driver.setPoseEstimate(startPose);
        hardReader.curPose = startPose;

        Trajectory wobbleA = driver.trajectoryBuilder(startPose)
                .lineToSplineHeading(field.WAI)
                .build();
        Trajectory preLA = driver.trajectoryBuilder(wobbleA.end())
                .lineToConstantHeading(field.PRLC.vec())
                .build();


        Trajectory wobbleB = driver.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-12, -50),0)
                .splineToSplineHeading(field.WBI,0)
                .build();
        Trajectory preLB = driver.trajectoryBuilder(wobbleB.end())
                .lineToConstantHeading(field.PRLC.vec())
                .build();

        Trajectory wobbleC = driver.trajectoryBuilder(startPose)
                .lineToSplineHeading(field.WCI)
                .build();
        Trajectory ppreLC = driver.trajectoryBuilder(wobbleC.end())
                .lineToConstantHeading(new Vector2d(field.WCII.getX()-10,field.align))
                .build();
        Trajectory preLC = driver.trajectoryBuilder(ppreLC.end())
                .lineToConstantHeading(new Vector2d(-6,field.align))
                .build();
        Trajectory LC = driver.trajectoryBuilder(preLC.end())
                .lineToConstantHeading(field.PRLC.vec())
                .build();

        Trajectory intakeI = driver.trajectoryBuilder(LC.end())
                .lineToConstantHeading(new Vector2d(-5.1, field.align), new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(2, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory intakeII = driver.trajectoryBuilder(intakeI.end())
                .lineToConstantHeading(new Vector2d(-20, field.align), new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(3, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory parkA = driver.trajectoryBuilder(preLA.end())
                //.splineToConstantHeading(new Vector2d(-10,30),0)
                .splineToConstantHeading(field.PAL.vec(),0)
                .build();
        Trajectory parkB = driver.trajectoryBuilder(intakeI.end())
                .lineToConstantHeading(field.PAM.vec())
                .build();
        Trajectory parkC = driver.trajectoryBuilder(intakeII.end())
                .lineToConstantHeading(field.PAM.vec())
                .build();


        hammer.grab();
        hammer.lift();
        telemetry.addData("Init","Initialized");
        telemetry.update();
        waitForStart();
        int stack = 0;
        wait vision = new wait(runtime, .5);
        shooter.doneReset();
        while (!vision.timeUp() && !isStopRequested() && opModeIsActive()) {
            stack = camera.height();
        }
        telemetry.addData("Stack height", stack);
        telemetry.update();
        roller.fallOut();
        runtime.reset();
        wait wobbleDrop = new wait(runtime,.2), wobbleGrab= new wait(runtime,.2);
        wait intakeTimer = new wait(runtime,.8);
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
                    if(stack==0) {
                        wobbleDrop = new wait(runtime, 5.8);
                    }
                    else{
                        wobbleDrop = new wait(runtime,.8);
                    }
                    if(!driver.isBusy()){
                        hammer.lowLift();
                        currentState = State.WD;
                    }
                    break;
                case WD:
                    if(wobbleDrop.timeUp()){
                        hammer.release();
                        hammer.lift();
                        switch(stack){
                            case 0:
                                //drive to shoot
                                driver.followTrajectoryAsync(preLA);
                                currentState = State.dTS;
                                break;
                            case 1:
                                //drive to shoot
                                driver.followTrajectoryAsync(preLB);
                                currentState = State.dTS;
                                break;
                            case 4:
                                //drive to intake
                                driver.followTrajectoryAsync(ppreLC);
                                currentState = State.dTTTS;
                        }
                        shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose,false));
                    }
                    break;
                case dTTTS:
                    if(!driver.isBusy()){
                        driver.followTrajectoryAsync(preLC);
                        currentState = State.dTTS;
                    }
                case dTTS:
                    if(!driver.isBusy()){
                        driver.followTrajectoryAsync(LC);
                        currentState = State.dTS;
                    }
                case dTS:
                    if(!driver.isBusy()){
                        currentState = State.S;
                        shooter.doneReset();
                    }
                    break;
                case S:
                    hammer.lift();
                    hammer.grab();
                    shooter.timedFireN(hardReader.shooterV); //turn on shooter during S
                    if(shooter.done){
                        shooter.doneReset();
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
                                intakeTimer = new wait(runtime,3);
                                roller.upToSpeed();
                                currentState = State.dTIS;
                                break;
                        }
                    }
                    break;
                case dTIS:
                    shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose,false));
                    if(!driver.isBusy()){
                        if(!intakeTimer.timeUp()) {
                            roller.upToSpeed(0);
                            currentState = State.IS;
                        }
                    }else{
                        intakeTimer = new wait(runtime,3);
                    }
                    break;
                case IS:
                    shooter.timedFireN(hardReader.shooterV); //turn on shooter during IS
                    if(shooter.done){
                        shooter.doneReset();
                        switch(stack){
                            case 1:
                                //park
                                driver.followTrajectoryAsync(parkB);
                                currentState = State.dTPA;
                                break;
                            case 4:
                                //drive to intake the stack, slow speed
                                shooter.doneReset();
                                driver.followTrajectoryAsync(intakeII);
                                roller.upToSpeed();
                                currentState = State.dTISI;
                                break;
                        }
                    }
                    break;
                case dTISI:
                    shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose,false));
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


