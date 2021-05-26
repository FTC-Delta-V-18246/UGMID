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
public class auto extends LinearOpMode {

    public SampleMecanumDrive driver;
    public reader hardReader;
    public hood shooter;
    public intake roller;
    public vision camera;
    public wobble hammer;
    public static double angle = 37;
    wait secondGrab, firstRelease, gap;
    wait powerShot;
    wait highShot;
    wait intake;
    enum State {
        POWER,
        INTAKEFOUR,
        INTAKEFOURA,
        INTAKEFOURB,
        INTAKEFOURC,
        STACK,
        HIGH,
        FIRSTWOBBLE,
        SECONDWOBBLE,
        IDLE
    }
    enum Rings{
        ZERO,
        ONE,
        FOUR,
    }


    private ElapsedTime runtime = new ElapsedTime();

    boolean POWER = false, HIGH = false;
    boolean pass = false;
    boolean park = false;
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void runOpMode() throws InterruptedException {

            double dropAAngle;
            double pickupAAngle;
            double dropBAngle;
            hardwareGenerator gen = new hardwareGenerator(this);
            subsystemGenerator subs = new subsystemGenerator(this, gen, runtime);
            this.shooter = subs.shooter;
            this.roller = subs.roller;
            this.driver = subs.driver;
            this.camera = subs.camera;
            this.hammer = subs.hammer;
            this.hardReader = subs.hardReader;

            FieldCoordinatesB field = new FieldCoordinatesB();
            Pose2d startPose = new Pose2d(-64,25,0);
            driver.setPoseEstimate(startPose);
            hardReader.curPose = startPose;
            Trajectory toPower = driver.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-36,18, Math.PI))
                    .build();

            Trajectory toIntake = driver.trajectoryBuilder(toPower.end().plus(new Pose2d(0,0,driver.turnToAbsolute(field.HR,toPower.end()))))

                        .addDisplacementMarker(() ->{
                            roller.upToSpeed();
                        })
                    .lineToLinearHeading(new Pose2d(-24,36, Math.PI/8))
                    .build();

        Trajectory intakeFOUR = driver.trajectoryBuilder(toPower.end().plus(new Pose2d(0,0,driver.turnToAbsolute(Math.PI/3.0,toPower.end()))))
                .addDisplacementMarker(()->{
                    shooter.feed();
                })
                .lineToLinearHeading(new Pose2d(-24.0,36.0, Math.PI/3))
                .build();
        Trajectory intakeFOURA = driver.trajectoryBuilder(intakeFOUR.end())
                .addDisplacementMarker(()->{
                    roller.upToSpeed();
                })
                .lineToLinearHeading(new Pose2d(-22.0,38.0, Math.PI/3))
                .build();
        Trajectory intakeFOURB = driver.trajectoryBuilder(intakeFOURA.end())
                .lineToLinearHeading(new Pose2d(-20.0,40.0, Math.PI/3))
                .build();
        Trajectory intakeFOURC = driver.trajectoryBuilder(intakeFOURB.end())
                .lineToLinearHeading(new Pose2d(-16.0,44.0, Math.PI/3))
                .build();


            Trajectory wobbleFOUR = driver.trajectoryBuilder(intakeFOURC.end().plus(new Pose2d(0,0,driver.turnToAbsolute(field.HR,intakeFOURC.end()))))
                    .lineToSplineHeading(new Pose2d(50,43,Math.PI/2.0))
                    .build();
            Trajectory wobbleFOURa = driver.trajectoryBuilder(wobbleFOUR.end())
                    .lineToLinearHeading(new Pose2d(-36.9, 38.6, Math.PI))
                    .build();
            Trajectory wobbleFOURb = driver.trajectoryBuilder(wobbleFOURa.end())
                    .lineToSplineHeading(new Pose2d(42,37,Math.PI/2.0))
                        .addDisplacementMarker(() -> {
                            //wobble release here
                            hammer.release();
                        })
                    .splineToSplineHeading(new Pose2d(5,30,Math.PI/2.0), 0)
                    .build();



            Trajectory wobbleONE = driver.trajectoryBuilder(toIntake.end().plus(new Pose2d(0,0,driver.turnToAbsolute(field.HR,toIntake.end()))))
                    .splineToSplineHeading(new Pose2d(22,43.2, 0), Math.toRadians(0))
                    .build();
            Trajectory wobbleONEa = driver.trajectoryBuilder(wobbleONE.end())
                    .lineToLinearHeading(new Pose2d(-37, 37, Math.PI))
                    .build();
            Trajectory wobbleONEb = driver.trajectoryBuilder(wobbleONEa.end())
                    .lineToSplineHeading(new Pose2d(16,39, 0))
                        .addDisplacementMarker(() -> {
                            //wobble release here
                            //hammer.release();
                        })
                    .build();
            Trajectory wobbleONEc = driver.trajectoryBuilder(wobbleONEb.end())
                    .lineToSplineHeading(new Pose2d(8,39,0))
                    .build();




            Trajectory wobbleZERO = driver.trajectoryBuilder(toPower.end().plus(new Pose2d(0,0,driver.turnToAbsolute(field.HR,toPower.end()))))
                    .splineToSplineHeading(new Pose2d(6,46.5,Math.PI/2.0), Math.toRadians(90)) //moving to drop first wobble
                    .build();
            Trajectory wobbleZEROa = driver.trajectoryBuilder(wobbleZERO.end())
                    .lineToLinearHeading(new Pose2d(-36.9, 38.72, Math.PI)) //moving to pick up second wobble
                    .build();
            Trajectory wobbleZEROb = driver.trajectoryBuilder(wobbleZEROa.end())
                    .lineToSplineHeading(new Pose2d(-5,40, Math.PI/2.0)) //moving to drop second wobble
                        .addDisplacementMarker(() -> {
                            //wobble release here
                            hammer.release();
                        })
                    .splineToLinearHeading(new Pose2d(5,20, Math.PI/2.0), Math.PI/2.0)
                    .build();


            hammer.grab();
            hammer.lift();
            int stack = camera.height();
            telemetry.addData("Stack height",stack);
            telemetry.addLine("Init Successful");
            telemetry.update();
            shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HR, hardReader.curPose, false));
            waitForStart();
            wait vision = new wait(runtime,.5);
                    while(!vision.timeUp()&&!isStopRequested()&&opModeIsActive()){
                        stack = camera.height();
                    }
            telemetry.addData("Stack height",stack);
            telemetry.update();
            roller.fallOut();
            runtime.reset();
            powerShot = new wait(runtime,0);
            intake = new wait(runtime,0);
            State currentState = State.POWER;
            driver.followTrajectoryAsync(toPower);
            hardReader.autonRead();
            while (!isStopRequested() && opModeIsActive()) {
                hardReader.autonRead();
                Pose2d curPose = hardReader.curPose;
                PoseStorage.currentPose = curPose;
                switch(currentState) {
                    case POWER:
                        hammer.grab();
                        if (!driver.isBusy()) {
                            if(!POWER){
                                shooter.raiseToAngle(-.01+shooter.calculateTargetShooterAngle(field.HR, hardReader.curPose, false));
                                driver.turnAsync(driver.turnToAbsolute(field.HR, curPose));
                                POWER = true;
                            } else if(!powerShot.timeUp()&&POWER){
                                shooter.fire(hardReader.shooterV);
                            }else if(POWER){
                                gen.pusherServo.setPosition(hood.leftPusherPos);
                                switch(stack){
                                    case 0:
                                        driver.followTrajectoryAsync(wobbleZERO);
                                        hammer.partialLift();
                                        roller.tuckIn();
                                        currentState = State.FIRSTWOBBLE;
                                        break;
                                    case 4:
                                        driver.turn(driver.turnToAbsolute(Math.PI/3,curPose));
                                        driver.followTrajectoryAsync(intakeFOUR);
                                        currentState = State.INTAKEFOUR;
                                        break;
                                    case 1:
                                        driver.turn(driver.turnToAbsolute(new Point(-24,36),curPose));
                                        driver.followTrajectoryAsync(toIntake);
                                        pass = true;
                                        currentState = State.STACK;
                                        break;

                                }
                            }
                        }else{
                            powerShot = new wait(runtime,1.7);
                        }
                        break;
                    case INTAKEFOUR:
                        if(!driver.isBusy()){
                            driver.followTrajectoryAsync(intakeFOURA);
                            currentState = State.INTAKEFOURA;
                            powerShot = new wait(runtime, .5);
                        }
                        break;
                    case INTAKEFOURA:
                        if(!driver.isBusy()&&powerShot.timeUp()){
                            driver.followTrajectoryAsync(intakeFOURB);
                            currentState = State.INTAKEFOURB;
                            powerShot = new wait(runtime, 1);
                        }
                        break;
                    case INTAKEFOURB:
                        if(!driver.isBusy()&&powerShot.timeUp()){
                            driver.followTrajectoryAsync(intakeFOURC);
                            currentState = State.INTAKEFOURC;
                            powerShot = new wait(runtime, .5);
                        }
                        break;
                    case INTAKEFOURC:
                        if(!driver.isBusy()&&powerShot.timeUp()){
                            hammer.partialLift();
                            currentState = State.STACK;
                        }
                        break;
                    case STACK:
                        if (!driver.isBusy()&&pass) {
                            if (intake.timeUp()) {
                                switch (stack) {
                                    case 1:
                                        driver.turn(driver.turnToAbsolute(field.HR,curPose)-Math.PI/6.0+Math.PI/90.0);
                                        break;
                                    case 4:
                                        driver.turn(driver.turnToAbsolute(field.HR,curPose)-Math.PI/6.0+Math.PI/45.0+Math.PI/18.0);
                                        break;
                                }
                                shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HR, hardReader.curPose, false));
                                roller.tuckIn();
                                currentState = State.HIGH;
                            }
                        }
                        else {
                            intake = new wait(runtime, 1.5);
                            pass = true;
                            shooter.feed();
                        }
                        break;
                    case HIGH:
                        hammer.partialLift();
                        roller.upToSpeed(0);
                        if(!HIGH){
                            HIGH = true;
                            gen.pusherServo.setPosition(hood.leftPusherPos);
                            powerShot = new wait(runtime, stack*.3+.8);
                        }
                        if(!powerShot.timeUp()){
                            shooter.fire(hardReader.shooterV);
                        }else{
                            switch(stack){
                                case 1:
                                    driver.followTrajectoryAsync(wobbleONE);
                                    break;
                                case 4:
                                    driver.followTrajectoryAsync(wobbleFOUR);
                                    break;
                            }
                                    currentState = State.FIRSTWOBBLE;
                        }
                       break;
                    case FIRSTWOBBLE:
                        if(!driver.isBusy()){
                            if(firstRelease.timeUp()) {
                                switch (stack) {
                                    case 1:
                                        driver.followTrajectoryAsync(wobbleONEa);
                                        break;
                                    case 0:
                                        driver.followTrajectoryAsync(wobbleZEROa);
                                        break;
                                    case 4:
                                        driver.followTrajectoryAsync(wobbleFOURa);
                                        break;
                                }
                                firstRelease = new wait(runtime,1);
                                currentState = State.SECONDWOBBLE;
                            }
                            else{
                                hammer.partialLift();
                                hammer.release();
                            }
                        }
                        else{
                            firstRelease = new wait(runtime, .5);
                        }
                        break;
                    case SECONDWOBBLE:
                        if(firstRelease.timeUp()){
                            hammer.down();
                        }
                        if(!driver.isBusy()){
                            if(secondGrab.timeUp()) {
                                hammer.partialLift();
                                switch (stack) {
                                    case 1:
                                        driver.followTrajectoryAsync(wobbleONEb);
                                        break;
                                    case 0:
                                        driver.followTrajectoryAsync(wobbleZEROb);
                                        break;
                                    case 4:
                                        driver.followTrajectoryAsync(wobbleFOURb);
                                        break;
                                }
                                currentState = State.IDLE;
                            }
                            else{
                                hammer.grab();
                            }
                        }
                        else{
                            secondGrab = new wait(runtime, .7);
                        }
                        break;
                    case IDLE:
                        if(!driver.isBusy()) {
                            if(secondGrab.timeUp()){
                                subs.hammer.grab();
                                subs.hammer.lift();
                                if(!park&&stack == 1){
                                    driver.followTrajectoryAsync(wobbleONEc);
                                    park = true;
                                }
                            }else{
                                subs.hammer.release();
                            }
                        }
                        else{
                            secondGrab = new wait(runtime, .7 );
                        }
                        break;

                }
                if(currentState== State.POWER || currentState == State.HIGH){
                    shooter.upToSpeed(hardReader.shooterV,runtime.seconds());
                }
                else{
                    shooter.safetySwitch(hardReader.shooterV,runtime.seconds());
                }
                driver.update();
                telemetry.addData("At velo?", shooter.atSpeed(hardReader.shooterV));
                telemetry.addData("Velo",hardReader.shooterV);
                telemetry.update();

            }
        }

    }

