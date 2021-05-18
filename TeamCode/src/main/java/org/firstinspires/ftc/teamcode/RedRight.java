
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@Autonomous(name = "RedRight", group = "Taus")
public class RedRight extends AutonomousMethods {

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
                forward(.5, 2, 10);
                toAngle(145, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(0, 1);
                break;
            case 1:
                //code
                forward(.5, 2, 16);
                shoot(-33.0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 21);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                backward(.5, 2, 21);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 12);
                forward(.5, 0, 12);
                toAngle(0, 1);
                forward(.5, 0, 23);
                shoot(-36.0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
            case 4:
                //code
                forward(.5, 2, 16);
                shoot(-33.0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 18);
                toAngle(145, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(180, 1);
                forward(.5, 2, 17);
                strafeLeft(.5, 0, 6);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 8);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 6);
                shoot(-36.0, (2360.9*28)/60.0);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 10);
                forward(.5, 0, 10);
                forward(.5, 0, 2);
                backward(.5, 0, 1);
                toAngle(0, 1);
                backward(.5, 1, 0);
                shoot(-33.0, (2215.7*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
        }

    }
}
