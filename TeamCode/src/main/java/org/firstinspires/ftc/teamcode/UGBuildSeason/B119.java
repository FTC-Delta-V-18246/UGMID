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
import org.firstinspires.ftc.teamcode.utilnonrr.FieldCoordinatesB;

import java.util.Arrays;

@Autonomous
@Config
public class B119 extends LinearOpMode {

    public SampleMecanumDrive driver;
    public reader hardReader;
    public hood shooter;
    public intake roller;
    public vision camera;
    public wobble hammer;
    public static double feeder = .3;

    private enum State {
        wobbleF,
        power,
        stack,
        intakeS,
        intakeT,
        intakeO,
        wobbleG,
        wobbleH,
        idle,
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
        Pose2d startPose = new Pose2d(-64, 49, 0);
        driver.setPoseEstimate(startPose);
        hardReader.curPose = startPose;
        Trajectory wobbleAA = driver.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-2, 63))
                .build();
        Trajectory wobbleBA = driver.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-12, 50),0)
                .splineToConstantHeading(new Vector2d(21, 39),0)
                .build();
        Trajectory wobbleCA = driver.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(44, 62))
                .build();

        Trajectory powerLA = driver.trajectoryBuilder(wobbleAA.end())
                .lineToConstantHeading(new Vector2d(-9, 36))
                .build();
        Trajectory powerLB = driver.trajectoryBuilder(wobbleBA.end())
                .lineToConstantHeading(new Vector2d(-9, 36))
                .build();
        Trajectory powerLC = driver.trajectoryBuilder(wobbleCA.end())
                .lineToConstantHeading(new Vector2d(-9, 36))
                .build();

        Trajectory powerM = driver.trajectoryBuilder(powerLA.end())
                .lineToConstantHeading(new Vector2d(-10, 15))
                .build();
        Trajectory powerR = driver.trajectoryBuilder(powerM.end())
                .lineToConstantHeading(new Vector2d(-10, 11))
                .build();

        Trajectory intakeS = driver.trajectoryBuilder(powerLC.end())
                .lineToConstantHeading(new Vector2d(-7, 36))
                .build();
        Trajectory intakeT = driver.trajectoryBuilder(intakeS.end())
                .lineToConstantHeading(new Vector2d(-7.1, 36), new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(2, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory intakeO = driver.trajectoryBuilder(intakeT.end())
                .lineToConstantHeading(
                        new Vector2d(-20, 36),
                        new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(11, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory wobbleAB = driver.trajectoryBuilder(powerLC.end())
                .lineToSplineHeading(new Pose2d(-32, 26, Math.PI))
                .build();
        Trajectory wobbleBB = driver.trajectoryBuilder(intakeT.end())
                .lineToSplineHeading(new Pose2d(-32, 26, Math.PI))
                .build();
        Trajectory wobbleCB = driver.trajectoryBuilder(intakeO.end())
                .lineToSplineHeading(new Pose2d(-30, 24, Math.PI))
                .build();

        Trajectory wobbleAC = driver.trajectoryBuilder(wobbleAB.end())
                .lineToSplineHeading(new Pose2d(-12, 60, 0))
                .build();
        Trajectory wobbleBC = driver.trajectoryBuilder(wobbleBB.end())
                .lineToSplineHeading(new Pose2d(15, 38, 0))
                .build();
        Trajectory wobbleCC = driver.trajectoryBuilder(wobbleCB.end())
                .lineToSplineHeading(new Pose2d(35, 62, 0))
                .build();

        Trajectory parkA = driver.trajectoryBuilder(wobbleAC.end())
                .splineToConstantHeading(new Vector2d(-20,30),0)
                .splineToConstantHeading(new Vector2d(10, 30),0)
                .build();
        Trajectory parkB = driver.trajectoryBuilder(wobbleBC.end())
                .lineToConstantHeading(new Vector2d(3, 30))
                .build();
        Trajectory parkC = driver.trajectoryBuilder(wobbleCC.end())
                .lineToConstantHeading(new Vector2d(10, 30))
                .build();


        hammer.grab();
        hammer.lift();
       // hammer.down();
        int stack = camera.height();
        telemetry.addData("Stack height", stack);
        telemetry.addLine("Init Successful");
        telemetry.update();
        //shooter.raiseToAngle(.3);//shooter.calculateTargetShooterAngle(field.HR, hardReader.curPose, false));
        waitForStart();
        wait vision = new wait(runtime, .5);
        wait timer3 = new wait(runtime,0);
        while (!vision.timeUp() && !isStopRequested() && opModeIsActive()) {
            stack = camera.height();
        }
        boolean intakeReady = false;
        //stack = 4;
        telemetry.addData("Stack height", stack);
        telemetry.update();
        roller.fallOut();
        runtime.reset();
        timer = new wait(runtime, 0);
        timer2 = new wait(runtime, 0);
        State currentState = State.wobbleF;
        switch (stack) {
            case 0:
                driver.followTrajectoryAsync(wobbleAA);
                break;
            case 1:
                driver.followTrajectoryAsync(wobbleBA);
                break;
            case 4:
                driver.followTrajectoryAsync(wobbleCA);
                break;
        }
        hardReader.autonRead();
        int shots = 0;
        int shooting = 1;
        while (!isStopRequested() && opModeIsActive()) {
            hardReader.autonRead();
            Pose2d curPose = hardReader.curPose;
            PoseStorage.currentPose = curPose;
            switch (currentState) {
                case wobbleF:
                    subs.angler.toPosition(.29);
                    hammer.down();
                    if (!driver.isBusy()) {
                        hammer.down();
                        if (!timer.timeUp()) {
                            //if timer is up, wobble releases && next path starts
                            switch (stack) {
                                case 0:
                                    driver.followTrajectoryAsync(powerLA);
                                    break;
                                case 1:
                                    driver.followTrajectoryAsync(powerLB);
                                    break;
                                case 4:
                                    driver.followTrajectoryAsync(powerLC);
                                    break;
                            }
                            hammer.release();
                            currentState = State.power;
                        }
                    } else {
                        //timer starts here for drop
                        timer = new wait(runtime, .4);
                    }
                    break;
                case power:
                    hammer.lift();
                    shooter.raiseToAngle(.02+shooter.calculateTargetShooterAngle(field.PM, hardReader.curPose, false));
                    if (!driver.isBusy()) {
                        if(!intakeReady&&shooter.atSpeed(hardReader.shooterV)&&!timerDone) {
                            timer = new wait(runtime, 1.6);
                            timerDone = true;
                        }
                        if(!timer.timeUp()) {
                            shooter.timedFireN(hardReader.shooterV);
                            roller.moveIt();
                        }
                        else if(timerDone){
                                    shooter.raiseToAngle(feeder);
                                    gen.pusherServo.setPosition(hood.leftPusherPos);
                                    switch (stack) {
                                        case 0:
                                            driver.followTrajectoryAsync(wobbleAB);
                                            hammer.down();
                                            currentState = State.wobbleG;
                                            break;
                                        case 1:
                                        case 4:
                                            //shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose, false));
                                            if(!intakeReady) {
                                                intakeReady = true;
                                                driver.followTrajectoryAsync(intakeS);
                                            }
                                            else{
                                                    roller.fallOut();
                                                    driver.followTrajectoryAsync(intakeT);
                                                    //timer = new wait(runtime,1.2);
                                                    roller.upToSpeed(1);
                                                    timer3 = new wait(runtime,3);
                                                    currentState = State.stack;
                                            }
                                            break;
                                    }


                            }
                    }
                    break;
                case stack:
                    shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose,false));
                    if (!driver.isBusy()&&timer3.timeUp()) {
                        if (timer.timeUp()) {
                            if(!timer2.timeUp()){
                                shooter.timedFireN(hardReader.shooterV);
                            }else {
                                switch (stack) {
                                    case 1:
                                        roller.upToSpeed(0);
                                        driver.followTrajectoryAsync(wobbleBB);
                                        subs.angler.toPosition(feeder);
                                        hammer.down();
                                        currentState = State.wobbleG;
                                        break;
                                    case 4:
                                        if (!shotFired) {
                                            gen.pusherServo.setPosition(hood.leftPusherPos);
                                            roller.tuckIn();
                                            driver.followTrajectoryAsync(intakeO);
                                            shotFired = true;
                                        } else {roller.upToSpeed(0);
                                            driver.followTrajectoryAsync(wobbleCB);
                                            subs.angler.toPosition(feeder);
                                            hammer.down();
                                            currentState = State.wobbleG;
                                        }
                                        break;
                                }
                            }
                        }else if(!timer.timeUp()){
                            timer2 = new wait(runtime,1.5);
                        }

                    }else{
                        roller.fallOut();
                        gen.pusherServo.setPosition(hood.leftPusherPos);
                        //if(shotFired){
                            timer2 = new wait(runtime,.5);
                            timer = new wait(runtime, 1.5);
                        //}
                    }
                    break;
                case wobbleG:
                    if (!driver.isBusy()) {
                        hammer.grab();
                        if (timer.timeUp()) {
                            hammer.lift();
                            switch (stack) {
                                case 0:
                                    driver.followTrajectoryAsync(wobbleAC);
                                    break;
                                case 1:
                                    driver.followTrajectoryAsync(wobbleBC);
                                    break;
                                case 4:
                                    driver.followTrajectoryAsync(wobbleCC);
                                    break;
                            }
                            currentState = State.wobbleH;
                        }
                    } else {
                        timer = new wait(runtime, 1);
                    }

                    break;
                case wobbleH:
                    if (!driver.isBusy()) {
                        hammer.lowLift();
                        if (timer.timeUp()) {
                            hammer.release();
                            switch (stack) {
                                case 0:
                                    driver.followTrajectoryAsync(parkA);
                                    break;
                                case 1:
                                    driver.followTrajectoryAsync(parkB);
                                    break;
                                case 4:
                                    driver.followTrajectoryAsync(parkC);
                                    break;
                            }
                            currentState = State.idle;
                        }
                    } else {
                        timer = new wait(runtime, .5);
                    }
                    break;
                case idle:
                    if (!driver.isBusy()) {
                        hammer.grab();
                        hammer.lift();
                        roller.fallOut();
                    }
                    break;
            }
                    if (currentState == State.power || currentState == State.stack) {
                        shooter.upToSpeed(hardReader.shooterV, runtime.seconds());
                        //gen.flyWheelM.setPower(1);
                        //gen.flyWheelM1.setPower(1);
                    } else {
                       // shooter.safetySwitch(hardReader.shooterV, runtime.seconds());
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


