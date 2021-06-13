package org.firstinspires.ftc.teamcode.UGBuildSeason;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class retiredAuto {

                        /*
                        if(!driver.isBusy()) {
                            roller.upToSpeedO(0);
                            roller.upToSpeedI(0);
                            switch (stack) {
                                case 1:
                                    if(shots<1) {
                                        if (powerShot.timeUp()) {
                                            if (shooter.fire(hardReader.shooterV)) {
                                                shots++;
                                                powerShot = new wait(runtime, .5);
                                            } else {
                                                gen.pusherServo.setPosition(hood.leftPusherPos);
                                            }
                                        }
                                    }
                                    else{
                                        if(powerShot.timeUp()) {
                                            driver.followTrajectoryAsync(wobbleONE);
                                            currentState = State.FIRSTWOBBLE;
                                        }
                                    }

                                break;
                                case 4:
                                    if(powerShot.timeUp()) {
                                        if(shots<3){
                                            if(shooter.fire(hardReader.shooterV)) {
                                                shots++;
                                                powerShot = new wait(runtime, .5);
                                            }
                                            else {
                                                gen.pusherServo.setPosition(hood.leftPusherPos);
                                            }
                                        }else{
                                            driver.followTrajectoryAsync(wobbleFOUR);
                                            currentState = State.FIRSTWOBBLE;
                                        }
                                    }

                                break;
                            }
                        }


                        break;

                         */
        /*
                            switch (shots) {
                                case 0:
                                    if(powerShot.timeUp()) {
                                        if (left_POWER) {
                                            if (shooter.fire(hardReader.shooterV)) {
                                                powerShot = new wait(runtime, .45);
                                                shots++;
                                            }
                                        } else {
                                            gen.pusherServo.setPosition(hood.leftPusherPos);
                                            shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HR, hardReader.curPose, false));
                                            driver.turnAsync(driver.turnToAbsolute(field.HR, curPose));
                                            left_POWER = true;
                                            telemetry.addData("Angle Change", shooter.turnToAbsolute(field.HR, hardReader.curPose));
                                            telemetry.update();
                                        }
                                    }else{
                                        gen.pusherServo.setPosition(hood.leftPusherPos);
                                    }
                                    break;
                                case 1:
                                    if(powerShot.timeUp()) {
                                        if (middle_POWER){
                                            if (shooter.fire(hardReader.shooterV)) {
                                                powerShot = new wait(runtime, .45);
                                                shots++;
                                            }
                                        } else {
                                            gen.pusherServo.setPosition(hood.leftPusherPos);
                                            //shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HR, hardReader.curPose, false));
                                            //driver.turnAsync(driver.turnToAbsolute(field.HR, curPose));
                                            middle_POWER = true;
                                            telemetry.addData("Angle Change", shooter.turnToAbsolute(field.HR, hardReader.curPose));
                                            telemetry.update();
                                        }
                                    }else{
                                        gen.pusherServo.setPosition(hood.leftPusherPos);
                                    }
                                    break;
                                case 2:
                                    if(powerShot.timeUp()) {
                                        if (right_POWER) {
                                            if (shooter.fire(hardReader.shooterV)) {
                                                gap = new wait(runtime, .45);
                                                shots++;
                                            }
                                        }
                                        else{
                                            gen.pusherServo.setPosition(hood.leftPusherPos);
                                            //shooter.raiseToAngle(shooter.calculateTargetShooterAngle(field.HR, hardReader.curPose, false));
                                            //driver.turnAsync(driver.turnToAbsolute(field.HR, curPose));
                                            right_POWER = true;
                                            telemetry.addData("Angle Change", shooter.turnToAbsolute(field.HR, hardReader.curPose));
                                            telemetry.update();
                                        }
                                    }else{
                                        gen.pusherServo.setPosition(hood.leftPusherPos);
                                    }
                                    break;
                                case 3:
                                    if(gap.timeUp()){
                                        gen.pusherServo.setPosition(hood.leftPusherPos);
                                        switch(stack){
                                            case 0:
                                                driver.followTrajectoryAsync(wobbleZERO);
                                                hammer.partialLift();
                                                roller.tuckIn();
                                                currentState = State.FIRSTWOBBLE;
                                                break;
                                            case 4:
                                                driver.turn(Math.PI/3);
                                                driver.followTrajectoryAsync(intakeFOUR);
                                                currentState = State.INTAKEFOUR;
                                                break;
                                            case 1:
                                                //driver.turn(new Point(-24,36));
                                                driver.followTrajectoryAsync(toIntake);
                                                pass = true;
                                                currentState = State.STACK;
                                                break;

                                        }
                                    }
                                    break;
                            }

                             */
}
