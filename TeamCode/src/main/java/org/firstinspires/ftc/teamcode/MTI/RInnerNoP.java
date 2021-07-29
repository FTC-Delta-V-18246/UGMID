package org.firstinspires.ftc.teamcode.MTI;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous
@Config
public class RInnerNoP extends LinearOpMode {
    private SampleMecanumDrive driver;
    private reader hardReader;
    private hood shooter;
    private intake roller;
    private vision camera;
    private wobble hammer;
    private stockTimer autoTimer;

    private ElapsedTime runtime = new ElapsedTime();

    private enum State {
        preLoad,
        park
    }
    Trajectory wobbleA, wobbleB, wobbleC, wobbleCI, wobbleBI, parkA, parkB, parkC;
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
        State currentState = State.preLoad;
        Pose2d startPose = new Pose2d(-64, 25, 0);
        driver.setPoseEstimate(startPose);
        hardReader.curPose = startPose;

        Trajectory pShot = driver.trajectoryBuilder(startPose)
                .lineToSplineHeading(field.PRLB)
                .build();
        wobbleA = driver.trajectoryBuilder(pShot.end())
                .lineToSplineHeading(field.WAIB)
                .addDisplacementMarker(() ->
                        hammer.down())
                .addDisplacementMarker(() ->
                        hammer.release())
                .addDisplacementMarker(() ->
                        driver.followTrajectoryAsync(parkA))
                .build();
        parkA = driver.trajectoryBuilder(wobbleA.end())
                .lineToSplineHeading(FieldCoordinatesR.PAL)
                .build();

        wobbleB = driver.trajectoryBuilder(pShot.end())
                .splineToLinearHeading(new Pose2d(50,15,0),0)
                .addDisplacementMarker(() ->
                        driver.followTrajectoryAsync(wobbleBI))
                .build();
        wobbleBI = driver.trajectoryBuilder(wobbleB.end())
                .splineToLinearHeading(FieldCoordinatesR.WBIB,0)
                .addDisplacementMarker(() ->
                        hammer.down())
                .addDisplacementMarker(() ->
                        hammer.release())
                .addDisplacementMarker(() ->
                        driver.followTrajectoryAsync(parkB))
                .build();
        parkB = driver.trajectoryBuilder(wobbleBI.end())
                .lineToSplineHeading(FieldCoordinatesR.PAL)
                .build();
        wobbleC = driver.trajectoryBuilder(pShot.end())
                .splineToLinearHeading(new Pose2d(30,30,Math.PI),0)
                .addDisplacementMarker(() ->
                        driver.followTrajectoryAsync(wobbleCI))
                .build();
        wobbleCI = driver.trajectoryBuilder(wobbleC.end())
                .splineToLinearHeading(field.WCIB,0)
                .addDisplacementMarker(() ->
                        hammer.down())
                .addDisplacementMarker(() ->
                        hammer.release())
                .addDisplacementMarker(() ->
                    driver.followTrajectoryAsync(parkC))
                .build();
        parkC = driver.trajectoryBuilder(wobbleCI.end())
                .lineToSplineHeading(FieldCoordinatesR.PAL)
                .build();

        Trajectory park = driver.trajectoryBuilder(wobbleCI.end())
                .lineToSplineHeading(field.PAL)
                .build();

        roller.tuckIn();
        shooter.liftDown();
        hammer.preload();
        int stack = 0;
        while(!opModeIsActive()&&!isStopRequested()){
            stack = camera.height();
            telemetry.addData("Stack height", stack);
            telemetry.update();
        }
        boolean turned = false;

        shooter.timedCancel();
        runtime.reset();

        boolean aligned = false;
        boolean noRepeat = false;

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
                case preLoad:
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
                                        hammer.lowLift();
                                        currentState = State.park;
                                        break;
                                    case 1:
                                        // drive to intake stack
                                        driver.followTrajectoryAsync(wobbleB);
                                        hammer.lowLift();
                                        currentState = State.park;
                                        break;
                                    case 4:
                                        // drive to intake stack
                                        driver.followTrajectoryAsync(wobbleC);
                                        hammer.lowLift();
                                        currentState = State.park;
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
                case park:
                    if(!driver.isBusy()){
                        shooter.liftDown();
                        hammer.lift();
                    }
                    break;
            }
            driver.update();


            if(currentState == State.preLoad){
                shooter.upToSpeed(hardReader.shooterV, runtime.seconds());
            }
            else{
                shooter.safetySwitch();
            }
        }
    }
}
