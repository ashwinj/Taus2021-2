
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "BlueRightMax", group = "Taus")
public class BlueRightMax extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(0);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 16);
                shoot(-34.0, (2215.0*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 21);
                strafeLeft(.5, 0, 23);
                dropWobbleGoal();
                backward(.5, 2, 0);
                toAngle(-65, 1);
                pickUpWobbleGoal(6);
                toAngle(0, 1);
                forward(.5, 1, 18);
                dropWobbleGoal();
                break;
            case 1:
                //code
                forward(.5, 2, 16);
                shoot(-33.0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 21);
                dropWobbleGoal();
                backward(.5, 1, 9);
                toAngle(30, 1);
                setIntakePower(1);
                backward(.5, 1, 19);
                toAngle(-60, 1);
                pickUpWobbleGoal(7);
                toAngle(0, 1);
                forward(.5, 1, 9);
                shoot(-10.0, (2212.4*28)/60.0);
                forward(.5, 0, 23);
                toAngle(170, 1);
                dropWobbleGoal();
                forward(.5, 0, 11);
                toAngle(0, 1);

                break;
            case 4:
                //code
                forward(.5, 2, 16);
                shoot(-33.0, (2214.2*28)/60.0);
                forward(.5, 2, 12);
                strafeLeft(.5, 0, 23);
                dropWobbleGoal();
                toAngle(-4, 1);
                setIntakePower(1);
                backward(.5, 2, 23);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 4);
                shoot(-19.0, (2272.4*28)/60.0);
                toAngle(-5, 1);
                backward(.5, 1, 0);
                toAngle(-40, 1);
                pickUpWobbleGoal(3);
                toAngle(0, 1);
                forward(.5, 1, 11);
                shoot(-15.0, (2212.8*28)/60.0);
                toAngle(0, 1);
                backward(.5, 2, 12);
                dropWobbleGoal();
                backward(.5, 2, 0);
                break;
        }

    }
}
