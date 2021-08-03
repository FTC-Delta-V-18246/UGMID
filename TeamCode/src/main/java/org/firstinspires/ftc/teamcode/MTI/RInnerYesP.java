package org.firstinspires.ftc.teamcode.MTI;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.subSystems.stockTimer;
import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator;
import org.firstinspires.ftc.teamcode.subSystems.vision;
import org.firstinspires.ftc.teamcode.subSystems.wobble;
import org.firstinspires.ftc.teamcode.util.wait;

import java.util.Arrays;

@Autonomous
@Config
public class RInnerYesP extends LinearOpMode {
    private SampleMecanumDrive driver;
    private reader hardReader;
    private hood shooter;
    private intake roller;
    private vision camera;
    private wobble hammer;
    private stockTimer autoTimer;

    private ElapsedTime runtime = new ElapsedTime();

    private enum State {
        powerShot,
        bounceBack,
        recovery,
        park
    }
    private enum Powershot{
        turnL,
        left,
        turnM,
        middle,
        turnR,
        right,
        end
    }
    Trajectory bb2, bb3, wobbleA, wobbleB, wobbleC, wobbleCI, wobbleTraj,wobbleBI, recoveryBI, recoveryCI;
    int stack = 0;

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
        this.autoTimer = subs.autoTimer;

        FieldCoordinatesR field = new FieldCoordinatesR();
        State currentState = State.powerShot;
        Powershot currentShot = Powershot.turnL;
        Pose2d startPose = new Pose2d(-64, -25, 0);
        driver.setPoseEstimate(startPose);
        hardReader.curPose = startPose;

        Trajectory pShot = driver.trajectoryBuilder(startPose)
                .lineToSplineHeading(field.PRLB)
                .build();
        Trajectory bb1 = driver.trajectoryBuilder(pShot.end())
                .splineToLinearHeading(new Pose2d(50,-10,-Math.PI/8.0),0)
                .addDisplacementMarker(() -> driver.followTrajectoryAsync(bb2))
                .build();
        bb2 = driver.trajectoryBuilder(bb1.end())
                .splineToLinearHeading(new Pose2d(50,-30,-Math.PI/2.0),0)
                .addDisplacementMarker(() -> driver.followTrajectoryAsync(bb3))
                .build();
        bb3 = driver.trajectoryBuilder(bb2.end())
                .splineToLinearHeading(new Pose2d(50,-35,-Math.PI/2.0),0)
                .addDisplacementMarker(() ->
                    driver.followTrajectoryAsync(wobbleTraj))
                .build();


        wobbleA = driver.trajectoryBuilder(bb3.end())
                .lineToSplineHeading(field.WAIB)
                .addDisplacementMarker(() ->
                        hammer.down())
                .build();

        wobbleB = driver.trajectoryBuilder(bb3.end())
                .splineToLinearHeading(new Pose2d(50,-15,0),0)
                .addDisplacementMarker(() ->
                        driver.followTrajectoryAsync(wobbleBI))
                .build();
        wobbleBI = driver.trajectoryBuilder(wobbleB.end())
                .splineToLinearHeading(FieldCoordinatesR.WBIB,0)
                .addDisplacementMarker(() ->
                        hammer.down())
                .build();
        wobbleC = driver.trajectoryBuilder(bb3.end())
                .lineToLinearHeading(new Pose2d(30,-20,Math.PI))
                .addDisplacementMarker(() ->
                        driver.followTrajectoryAsync(wobbleCI))
                .build();
        wobbleCI = driver.trajectoryBuilder(wobbleC.end())
                .splineToLinearHeading(field.WCIB,0)
                .addDisplacementMarker(() ->
                        hammer.down())
                .build();
        Trajectory recoveryA = driver.trajectoryBuilder(wobbleA.end())
                .splineToLinearHeading(new Pose2d(35,-18,0),0)
                .splineToLinearHeading(field.PRLB,0,new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(.7*DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() ->
                        driver.turnAsync(field.HM))
                .build();
        Trajectory recoveryB = driver.trajectoryBuilder(wobbleB.end())
                .splineToLinearHeading(new Pose2d(26,-16,Math.PI/4.0),0)
                .addDisplacementMarker(() ->
                        driver.followTrajectoryAsync(recoveryBI))
                .build();
        recoveryBI = driver.trajectoryBuilder(recoveryB.end())
                .splineToLinearHeading(field.PRLB,0,new MinVelocityConstraint(
                Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(.7*DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() ->
                        driver.turnAsync(field.HM))
                .build();
        Trajectory recoveryC = driver.trajectoryBuilder(wobbleC.end())
                .splineToLinearHeading(new Pose2d(35,-18,0),0)
                .addDisplacementMarker(() ->
                        driver.followTrajectoryAsync(recoveryCI))
                .build();
        recoveryCI = driver.trajectoryBuilder(recoveryC.end())
                .splineToLinearHeading(field.PRLB,0,new MinVelocityConstraint(
                        Arrays.asList(new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),new MecanumVelocityConstraint(.7*DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() ->
                        driver.turnAsync(field.HM))
                .build();

        Trajectory park = driver.trajectoryBuilder(recoveryC.end())
                .lineToSplineHeading(field.PAL)
                .build();

        boolean turned = false;
        roller.tuckIn();
        shooter.liftDown();
        hammer.preload();
        int stack = 0;
        while(!opModeIsActive()&&!isStopRequested()){
            stack = camera.height();
            telemetry.addData("Stack height", stack);
            telemetry.update();
        }
        switch(stack) {
            case 0:
                wobbleTraj = wobbleA;
                break;
            case 1:
                wobbleTraj = wobbleB;
                break;
            case 4:
                wobbleTraj = wobbleC;
                break;
        }

        roller.tuckIn();
        shooter.timedCancel();
        runtime.reset();

        boolean noRepeat = false;
        boolean secondIntake = false;

        wait lift = autoTimer.lift;
        wait liftD = autoTimer.liftD;
        wait preload = autoTimer.preload;
        wait shotTimer = autoTimer.shotTimer;
        wait retract = autoTimer.retract;
        autoTimer.start();
        lift.init();
        shotTimer.init();
        driver.followTrajectoryAsync(pShot);
        while (!isStopRequested() && opModeIsActive()) {
            hardReader.autonRead();
            Pose2d curPose = hardReader.curPose;
            PoseStorage.currentPose = curPose;

            switch (currentState) {
                case powerShot:
                    shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose,true));
                    preload.init();
                    if(preload.timeUp()){
                        hammer.partialLift();
                        if(!noRepeat) {
                            lift.init();
                            shooter.liftUp();
                            noRepeat = true;
                        }
                    }

                    if(!driver.isBusy()&&lift.timeUp()&&preload.timeUp()&&lift.init) {
                        driver.setWeightedDrivePower(new Pose2d(0,0,0));
                        switch(currentShot){
                            case turnL:
                                driver.turnAsync(FieldCoordinatesR.PL);
                                currentShot = Powershot.left;
                                break;
                            case left:
                                if(!shotTimer.timeUp()&&shooter.atSpeed(hardReader.shooterV)){
                                    gen.pusherServo.setPosition(shooter.rightPusherPos);
                                    retract.init();
                                    if(retract.timeUp()){
                                        gen.pusherServo.setPosition(shooter.leftPusherPos);
                                    }
                                }
                                if(retract.timeUp()&&shotTimer.timeUp()){
                                    gen.pusherServo.setPosition(shooter.leftPusherPos);
                                    driver.turnAsync(FieldCoordinatesR.PM);
                                    retract.deinit();
                                    shotTimer.init();
                                    currentShot = Powershot.middle;
                                }
                                break;
                            case middle:
                                if(!shotTimer.timeUp()&&shooter.atSpeed(hardReader.shooterV)) {
                                    gen.pusherServo.setPosition(shooter.rightPusherPos);
                                    retract.init();
                                    if(retract.timeUp()){
                                        gen.pusherServo.setPosition(shooter.leftPusherPos);
                                    }
                                }
                                if(retract.timeUp()&&shotTimer.timeUp()){
                                        gen.pusherServo.setPosition(shooter.leftPusherPos);
                                        driver.turnAsync(field.PR);
                                        retract.deinit();
                                        shotTimer.init();
                                        currentShot = Powershot.right;
                                }
                                break;
                            case right:
                                if(!shotTimer.timeUp()&&shooter.atSpeed(hardReader.shooterV)) {
                                    gen.pusherServo.setPosition(shooter.rightPusherPos);
                                    retract.init();
                                    if(retract.timeUp()){
                                        gen.pusherServo.setPosition(shooter.leftPusherPos);
                                    }
                                }
                                if(shotTimer.timeUp() && retract.timeUp()){
                                        shooter.liftDown();
                                        if(liftD.timeUp()) {
                                            driver.followTrajectoryAsync(bb1);
                                            currentState = State.bounceBack;
                                            retract.deinit();
                                            roller.fallOut();
                                            roller.upToSpeed();
                                        }
                                }else{
                                        liftD.deinit();
                                        liftD.init();
                                    }
                                break;
                        }

                    }else{
                        shotTimer.init();
                    }
                    break;
                case bounceBack:
                    if(!driver.isBusy()) {
                        shooter.goalVelo = 21;
                        hammer.release();
                        hammer.partialLift();
                        lift.init();
                        roller.upToSpeed(0);
                        roller.tuckIn();
                        shooter.liftUp();
                        switch(stack) {
                            case 0:
                                driver.followTrajectoryAsync(recoveryA);
                                break;
                            case 1:
                                driver.followTrajectoryAsync(recoveryB);
                                break;
                            case 4:
                                driver.followTrajectoryAsync(recoveryC);
                                break;
                        }
                        currentState = State.recovery;
                    }
                    break;
                case recovery:
                    shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose,false));
                    if(!driver.isBusy()&&lift.timeUp()) {
                        shooter.timedFireN(hardReader.shooterV);
                        if (shooter.done) {
                            gen.pusherServo.setPosition(hood.leftPusherPos);
                            shooter.timedCancel();
                            shooter.liftDown();
                            lift.deinit();
                            driver.followTrajectoryAsync(park);
                            currentState = State.park;
                        }
                    }
                    break;
                case park:
                    if(!driver.isBusy()){
                        shooter.liftDown();
                    }
                    hammer.lift();
                    break;
            }
                driver.update();


            if(currentState == State.powerShot || currentState == State.recovery){
                shooter.upToSpeed(hardReader.shooterV, runtime.seconds());
            }
            else{
                shooter.safetySwitch();
            }
        }
    }
}
