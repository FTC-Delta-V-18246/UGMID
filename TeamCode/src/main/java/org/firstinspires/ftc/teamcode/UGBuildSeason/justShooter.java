package org.firstinspires.ftc.teamcode.UGBuildSeason;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//import com.arcrobotics.ftclib.vision.UGContourRingPipeline;


@TeleOp
@Config
public class justShooter extends LinearOpMode {



    @RequiresApi(api = Build.VERSION_CODES.N)
    public void runOpMode() {

        DcMotorEx flyWheelM;
        flyWheelM = hardwareMap.get(DcMotorEx.class, "fly_wheel");
        flyWheelM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheelM.setDirection(DcMotorEx.Direction.REVERSE);
        flyWheelM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        telemetry.addData("Init", "Successful");
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            //flyWheelM.setPower(1);
            double shooterV = flyWheelM.getVelocity()/(28.0)*.096* Math.PI;//(double)flyWheelM.getVelocity()/28.0;

            telemetry.addData("velo", shooterV);
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard dashboard = FtcDashboard.getInstance();
            packet.put("Target Velocity", 24);
            packet.put("Actual Velocity Shooter", shooterV);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}

