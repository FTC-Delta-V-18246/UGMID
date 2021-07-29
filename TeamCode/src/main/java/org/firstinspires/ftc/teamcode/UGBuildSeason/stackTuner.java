package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.subSystems.hardwareGenerator;
import org.firstinspires.ftc.teamcode.subSystems.subsystemGenerator;
import org.firstinspires.ftc.teamcode.subSystems.vision;

@Autonomous
@Config
public class stackTuner extends LinearOpMode {


    public vision camera;
    enum Rings{
        ZERO,
        ONE,
        FOUR,
    }
ElapsedTime runtime = new ElapsedTime();

    public static final String VUFORIA_LICENSE_KEY = "AWPLlov/////AAABmRUzpLRYdkiUv/AKPCnb40NhUmsrAmbAh675VMgclj/FQXPqeIV9MS3lHOukiZ1o4icOfVXAx18jq2whSMkg+pnFk2XrgajckmaxFXFIm09xn4tFM4W7fSUHUaa+U+ypbcoOHUi9O7ZS2Ums8h33xhx2y/euqxdbiS+TApiUUsQIHS4eJJrk4GcPAd9GzN5EmDEmujprST7qpi6/WfwGQn3MIxj5REU6N8yGhDbI6ME7Yyz0PsWY50d4iJRbPJZgIpJj9ZqIQS1MG58JQj5ZCwJP7j0b9pAOHUifs8OYNzcbFGxP07+wCaniA7BwJFCOoM/xZcv0iEhXbD6vURdLJyGxCPeE83arAdXiholx7DuU";
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void runOpMode() throws InterruptedException {


        hardwareGenerator gen = new hardwareGenerator(this);
        subsystemGenerator subs = new subsystemGenerator(this, gen, runtime);

        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);




        this.camera = subs.camera;


        int stack = camera.height();
        telemetry.addData("Stack height",stack);
        telemetry.addLine("Init Successful");
        telemetry.update();
        waitForStart();
        telemetry.addData("Stack height",stack);
        telemetry.update();
        runtime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            camera.updateColor();
            stack = camera.height();
            telemetry.addData("Stack height",stack);
            telemetry.update();
    }
    }

}

