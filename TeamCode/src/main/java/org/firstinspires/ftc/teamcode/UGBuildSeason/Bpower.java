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
import org.firstinspires.ftc.teamcode.subSystems.*;
import org.firstinspires.ftc.teamcode.util.wait;
import org.firstinspires.ftc.teamcode.utilnonrr.FieldCoordinatesB;

import java.util.Arrays;

@Autonomous
@Config
public class Bpower extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(-64,25, 0);
        driver.setPoseEstimate(startPose);
        hardReader.curPose = startPose;
        Trajectory power = driver.trajectoryBuilder(startPose)
                .lineToConstantHeading(field.PS.vec())
                .build();


        Trajectory wobbleA = driver.trajectoryBuilder(power.end())
                .lineToSplineHeading(new Pose2d(-30, 24, Math.PI))
                .build();
        Trajectory bounceBack0 = driver.trajectoryBuilder(wobbleA.end())
                .lineToSplineHeading(new Pose2d(-22, 36, Math.PI))
                .build();


        Trajectory wobbleB = driver.trajectoryBuilder(power.end())
                .lineToSplineHeading(new Pose2d(-30, 24, Math.PI))
                .build();
        Trajectory bounceBack1 = driver.trajectoryBuilder(wobbleB.end())
                .build();

        Trajectory wobbleC = driver.trajectoryBuilder(power.end())
                .lineToSplineHeading(new Pose2d(-30, 24, Math.PI))
                .build();
        Trajectory bounceBack4 = driver.trajectoryBuilder(wobbleB.end())
                .build();

        Trajectory parkA = driver.trajectoryBuilder(bounceBack0.end())
                .splineToConstantHeading(new Vector2d(-10,30),0)
                .splineToConstantHeading(new Vector2d(10, 30),0)
                .build();
        Trajectory parkB = driver.trajectoryBuilder(bounceBack1.end())
                .lineToConstantHeading(new Vector2d(10, 30))
                .build();
        Trajectory parkC = driver.trajectoryBuilder(bounceBack4.end())
                .lineToConstantHeading(new Vector2d(10, 30))
                .build();


        hammer.grab();
        hammer.lift();
        waitForStart();
        int stack = 0;
        wait vision = new wait(runtime, .5);
        while (!vision.timeUp() && !isStopRequested() && opModeIsActive()) {
            stack = camera.height();
        }
        telemetry.addData("Stack height", stack);
        telemetry.update();
        roller.fallOut();
        runtime.reset();
        wait wobbleDrop = new wait(runtime,.2), wobbleGrab= new wait(runtime,.2);;
        State currentState = State.dTPO;
        PowerShot powerS = PowerShot.L;
        driver.followTrajectoryAsync(power); //drive to powershots
        while (!isStopRequested() && opModeIsActive()) {
            hardReader.autonRead();
            Pose2d curPose = hardReader.curPose;
            PoseStorage.currentPose = curPose;
            switch (currentState) {
                case dTPO:
                    subs.angler.toPosition(.29);
                    if (!driver.isBusy()) {
                            currentState = State.P;
                    }
                    break;
                case P:
                    //turn shooter on during P
                    shooter.raiseToAngle(.02+shooter.calculateTargetShooterAngle(field.PM, hardReader.curPose, false));
                    if(!driver.isBusy()){
                        switch(powerS){
                            case L:
                                driver.turnAsync(field.PL);
                            break;
                            case M:
                                driver.turnAsync(field.PM);
                            break;
                            case R:
                                driver.turnAsync(field.PR);
                            break;
                        }
                    }
                    if(!driver.isBusy()){
                        shooter.fire(hardReader.shooterV);
                        switch(powerS){
                            case L:
                                powerS = PowerShot.M;
                                break;
                            case M:
                                powerS = PowerShot.R;
                                break;
                            case R:
                                roller.upToSpeed();
                                switch(stack){
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
                                currentState  = State.dTWD;
                                break;
                        }
                    }
                    break;
                case dTWD:
                    wobbleDrop = new wait(runtime,.2);
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
                                //bounceback then drive to shoot
                                driver.followTrajectoryAsync(bounceBack0);
                                break;
                            case 1:
                                //bounceback then drive to shoot
                                driver.followTrajectoryAsync(bounceBack1);
                                break;
                            case 4:
                                //bounceback then drive to shoot
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
                    if(shooter.done){
                        switch(stack){
                            case 0:
                                //park
                                driver.followTrajectoryAsync(parkA);
                                break;
                            case 1:
                                //park
                                driver.followTrajectoryAsync(parkB);
                                break;
                            case 4:
                                //park
                                driver.followTrajectoryAsync(parkC);
                                break;
                        }

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
                    if (currentState == State.S || currentState == State.P) {
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


