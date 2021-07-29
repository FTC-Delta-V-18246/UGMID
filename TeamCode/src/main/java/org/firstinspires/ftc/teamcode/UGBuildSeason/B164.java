package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.wait;
import org.firstinspires.ftc.teamcode.subSystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;
import org.firstinspires.ftc.teamcode.subSystems.hood;
import org.firstinspires.ftc.teamcode.subSystems.intake;
import org.firstinspires.ftc.teamcode.subSystems.reader;
import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator;
import org.firstinspires.ftc.teamcode.subSystems.vision;
import org.firstinspires.ftc.teamcode.subSystems.wobble;
import org.firstinspires.ftc.teamcode.MTI.FieldCoordinatesB;

@Disabled
@Autonomous
@Config
public class B164 extends LinearOpMode {

    public SampleMecanumDrive driver;
    public reader hardReader;
    public hood shooter;
    public intake roller;
    public vision camera;
    public wobble hammer;

    private enum State {
        dTPO,
        P,
        dTWD,
        WD,
        dTS,
        S,
        dTIS,
        IS,
        dTISI,
        ISI,
        dTWPI,
        WPI,
        dTWDI,
        WDI,
        dTPA,
        idle,
    }
    enum PowerShot{
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
        Pose2d startPose = new Pose2d(-64,24, 0);
        driver.setPoseEstimate(startPose);
        hardReader.curPose = startPose;
        Trajectory power = driver.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-9, 20))
                .build();


        Trajectory wobbleA = driver.trajectoryBuilder(power.end())
                .lineToConstantHeading(new Vector2d(-6, 61))
                .build();
        Trajectory bounceBack0 = driver.trajectoryBuilder(wobbleA.end())
                .lineToSplineHeading(new Pose2d(-22, 36, 0))
                .build();
        Trajectory wobbleAI = driver.trajectoryBuilder(bounceBack0.end())
                .lineToSplineHeading(new Pose2d(-29, 47.5, Math.PI+Math.toRadians(9)))
                .build();
        Trajectory wobbleADI = driver.trajectoryBuilder(wobbleAI.end())
                .lineToSplineHeading(new Pose2d(-12, 58, 0))
                .build();


        Trajectory wobbleB = driver.trajectoryBuilder(power.end())
                .lineToSplineHeading(new Pose2d(-30, 24, Math.PI))
                .build();
        Trajectory bounceBack1 = driver.trajectoryBuilder(wobbleB.end())
                .lineToConstantHeading(new Vector2d(-15, 36))
                .build();
        Trajectory intakeI = driver.trajectoryBuilder(bounceBack1.end())
                .lineToConstantHeading(new Vector2d(-22, 36))
                .build();
        Trajectory wobbleBI = driver.trajectoryBuilder(intakeI.end())
                .lineToSplineHeading(new Pose2d(-30, 24, Math.PI))
                .build();
        Trajectory wobbleBDI = driver.trajectoryBuilder(wobbleBI.end())
                .lineToSplineHeading(new Pose2d(15, 38, 0))
                .build();

        Trajectory wobbleC = driver.trajectoryBuilder(power.end())
                .lineToSplineHeading(new Pose2d(-30, 24, Math.PI))
                .build();
        Trajectory bounceBack4 = driver.trajectoryBuilder(wobbleB.end())
                .lineToConstantHeading(new Vector2d(-15, 36))
                .build();
        Trajectory intakeII = driver.trajectoryBuilder(intakeI.end())
                .lineToConstantHeading(new Vector2d(-30, 36))
                .build();
        Trajectory wobbleCI = driver.trajectoryBuilder(intakeII.end())
                .lineToSplineHeading(new Pose2d(-30, 24, Math.PI))
                .build();
        Trajectory wobbleCDI = driver.trajectoryBuilder(wobbleCI.end())
                .lineToSplineHeading(new Pose2d(35, 62, 0))
                .build();

        Trajectory parkA = driver.trajectoryBuilder(wobbleADI.end())
                .splineToConstantHeading(new Vector2d(-20, 55),0)
                .splineToConstantHeading(new Vector2d(1, 40),0)
                .build();
        Trajectory parkB = driver.trajectoryBuilder(wobbleBDI.end())
                .lineToConstantHeading(new Vector2d(10, 30))
                .build();
        Trajectory parkC = driver.trajectoryBuilder(wobbleCDI.end())
                .lineToConstantHeading(new Vector2d(10, 30))
                .build();


        hammer.grab();
        hammer.lift();
        waitForStart();
        int stack = 0;
        wait vision = new wait(runtime, .5);
        wait rShot = new wait(runtime,.1);
        while (!vision.timeUp() && !isStopRequested() && opModeIsActive()) {
            stack = camera.height();
        }
        telemetry.addData("Stack height", stack);
        telemetry.update();
        roller.fallOut();
        runtime.reset();
        wait wobbleDrop = new wait(runtime,.2), wobbleGrab= new wait(runtime,.2),pShot = new wait(runtime,.2);
        State currentState = State.dTPO;
        PowerShot powerS = PowerShot.L;
        driver.followTrajectoryAsync(power); //drive to powershots
        while (!isStopRequested() && opModeIsActive()) {
            hardReader.autonRead();
            Pose2d curPose = hardReader.curPose;
            PoseStorage.currentPose = curPose;
            switch (currentState) {
                case dTPO:
                    shooter.toPosition(.29);
                    if (!driver.isBusy()) {
                            driver.turnAsync(field.PL);
                            currentState = State.P;
                    }
                    break;
                case P:
                    //turn shooter on during P
                    shooter.raiseToAngle(.02+shooter.calculateTargetShooterAngle(field.PM, hardReader.curPose, false));
                    if(!driver.isBusy()){
                        switch(powerS){
                            case L:
                                if(shooter.shots!=0) {
                                    powerS = PowerShot.M;
                                    driver.turnAsync(field.PM);
                                }
                                break;
                            case M:
                                if(shooter.shots!=1) {
                                    powerS = PowerShot.R;
                                    driver.turnAsync(field.PR);
                                }
                                break;
                            case R:
                                if(shooter.shots!=2) {
                                    switch (stack) {
                                        case 0:
                                            //drive to zone A
                                            driver.followTrajectoryAsync(wobbleA);
                                            break;
                                        case 1:
                                            //drive to zone B
                                            driver.followTrajectoryAsync(wobbleB);
                                            break;
                                        case 4:
                                            //drive to zone C
                                            driver.followTrajectoryAsync(wobbleC);
                                            break;
                                    }
                                    currentState = State.dTWD;
                                }
                                break;
                        }
                        if(shooter.fire(hardReader.shooterV)&&pShot.timeUp()){
                            shooter.shots++;
                            pShot = new wait(runtime,.8);
                            rShot = new wait(runtime,.1);
                        };
                    }
                    if(rShot.timeUp()){
                        gen.pusherServo.setPosition(hood.leftPusherPos);
                    }
                    break;
                case dTWD:
                    wobbleDrop = new wait(runtime,.4);
                    if(!driver.isBusy()){
                        hammer.lowLift();
                        currentState = State.WD;
                    }
                break;
                case WD:
                    if(wobbleDrop.timeUp()){
                        hammer.release();
                        roller.upToSpeed(); //turn on roller for bounceback
                        switch(stack){
                            case 0:
                                //bounceback then drive to center
                                driver.followTrajectoryAsync(bounceBack0);
                                break;
                            case 1:
                                //bounceback then drive infront of stack
                                driver.followTrajectoryAsync(bounceBack1);
                                break;
                            case 4:
                                //bounceback then drive infront of stack
                                driver.followTrajectoryAsync(bounceBack4);
                        }
                        currentState = State.dTS;
                    }
                break;
                case dTS:
                    if(!driver.isBusy()){
                        roller.upToSpeed(0);
                        currentState = State.S;
                    }
                break;
                case S:
                    shooter.timedFireN(hardReader.shooterV); //turn on shooter during S
                    if(shooter.shots>2){
                        switch(stack){
                            case 0:
                                //drive to pickup wobble
                                hammer.down();
                                driver.followTrajectoryAsync(wobbleAI);
                                currentState = State.dTWPI;
                            break;
                            case 1:
                            case 4:
                                //drive to intake the stack, slow speed
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
                                //drive to pickup wobble
                                hammer.down();
                                currentState = State.dTWPI;
                            break;
                            case 4:
                                //drive to intake the stack, slow speed
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
                    if(shooter.shots>2){
                        //drive to pickup wobble
                        currentState = State.dTWPI;
                    }
                break;
                case dTWPI:
                    wobbleGrab = new wait(runtime, 1);
                    if(!driver.isBusy()){
                        hammer.grab();
                        currentState = State.WPI;
                    }
                break;
                case WPI:
                    if(wobbleGrab.timeUp()){
                        //drive to drop wobble
                        switch(stack){
                            case 0:
                                driver.followTrajectoryAsync(wobbleADI);
                                break;
                            case 1:
                                driver.followTrajectoryAsync(wobbleBDI);
                                break;
                            case 4:
                                driver.followTrajectoryAsync(wobbleCDI);
                                break;
                        }
                        currentState = State.dTWDI;
                    }
                break;
                case dTWDI:
                    if(!driver.isBusy()){
                        currentState = State.WDI;
                    }
                break;
                case WDI:
                   hammer.release();
                    //drive to park
                    switch(stack){
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
                    currentState = State.dTPA;
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
                    if (currentState == State.S || currentState == State.IS|| currentState == State.ISI|| currentState == State.P) {
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


