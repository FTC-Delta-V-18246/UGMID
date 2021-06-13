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
import org.firstinspires.ftc.teamcode.utilnonrr.FieldCoordinatesR;

@Autonomous
@Config
public class Rcomplex extends LinearOpMode {

    public SampleMecanumDrive driver;
    public reader hardReader;
    public hood shooter;
    public intake roller;
    public vision camera;
    public wobble hammer;

    private enum State {
        dTS,
        S,
        dTWD,
        WD,
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

        hardwareGenerator gen = new hardwareGenerator(this);
        subsystemGenerator subs = new subsystemGenerator(this, gen, runtime);
        this.shooter = subs.shooter;
        this.roller = subs.roller;
        this.driver = subs.driver;
        this.camera = subs.camera;
        this.hammer = subs.hammer;
        this.hardReader = subs.hardReader;

        FieldCoordinatesR field = new FieldCoordinatesR();
        Pose2d startPose = new Pose2d(-64,-25, 0);
        driver.setPoseEstimate(startPose);
        hardReader.curPose = startPose;

        Trajectory preL = driver.trajectoryBuilder(startPose)
                .lineToSplineHeading(field.PRLB)
                .build();

        Trajectory wobbleA = driver.trajectoryBuilder(preL.end())
                .lineToSplineHeading(field.WAI)
                .build();

        Trajectory wobbleB = driver.trajectoryBuilder(preL.end())
                .lineToSplineHeading(field.WBII)
                .build();

        Trajectory wobbleC = driver.trajectoryBuilder(preL.end())
                .lineToSplineHeading(field.WCII)
                .build();


        Trajectory parkA = driver.trajectoryBuilder(wobbleA.end())
                .lineToConstantHeading(field.PAL.vec())
                .build();
        Trajectory parkB = driver.trajectoryBuilder(wobbleB.end())
                .lineToConstantHeading(field.PAL.vec())
                .build();
        Trajectory parkC = driver.trajectoryBuilder(wobbleC.end())
                .lineToConstantHeading(field.PAL.vec())
                .build();


        hammer.grab();
        hammer.lift();
        telemetry.addData("Init","Initialized");
        telemetry.update();
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
        State currentState = State.dTS;
                driver.followTrajectoryAsync(preL);
        while (!isStopRequested() && opModeIsActive()) {
            hardReader.autonRead();
            Pose2d curPose = hardReader.curPose;
            PoseStorage.currentPose = curPose;
            switch (currentState) {
                case dTS:
                    shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HM, hardReader.curPose,false));
                    if(!driver.isBusy()){
                        driver.turnAsync(field.HM);
                        currentState = State.S;
                    }
                    break;
                case S:
                    if(!driver.isBusy()) {
                            shooter.timedFireN(hardReader.shooterV); //turn on shooter during S
                        if(shooter.done) {
                            switch (stack) {
                                case 0:
                                    //drive to wobble
                                    driver.followTrajectoryAsync(wobbleA);
                                    break;
                                case 1:
                                    //drive to wobble
                                    driver.followTrajectoryAsync(wobbleB);
                                    break;
                                case 4:
                                    //drive to wobble
                                    driver.followTrajectoryAsync(wobbleC);
                            }
                            currentState = State.dTWD;
                        }
                    }
                    break;
                case dTWD:
                    wobbleDrop = new wait(runtime,1);
                    if(!driver.isBusy()){
                        hammer.down();
                        currentState = State.WD;
                    }
                    break;
                case WD:
                    if(wobbleDrop.timeUp()){
                        hammer.release();
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
                        hammer.lift();
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
            if (currentState == State.S) {
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


