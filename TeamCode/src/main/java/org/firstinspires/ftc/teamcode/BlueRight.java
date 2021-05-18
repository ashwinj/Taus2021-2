
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@Autonomous(name = "BlueRight", group = "Taus")
public class BlueRight extends AutonomousMethods {

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
                forward(.5, 0, 20);
                strafeLeft(.5, 0, 23);
                dropWobbleGoal();
                strafeRight(.5, 1, 4);
                backward(.5, 0, 8);
                break;
            case 1:
                //code
                forward(.5, 2, 16);
                shoot(-33.0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 20);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                backward(.5, 2, 20);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 11);
                forward(.5, 0, 11);
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
                forward(.5, 2, 8);
                strafeLeft(.5, 0, 23);
                toAngle(45, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 11);
                toAngle(0, 1);
                backward(.5, 2, 7);
                toAngle(90, 1);
                backward(.5, 0, 6);
                setIntakePower(0);
                backward(.5, 0, 2);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 6);
                shoot(-35.0, (2360.3*28)/60.0);
                toAngle(90, 1);
                backward(.5, 0, 8);
                forward(.5, 0, 11);
                toAngle(0, 1);
                forward(.5, 0, 23);
                shoot(-35.0, (2216.3*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
        }

    }
}
