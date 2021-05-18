
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@Autonomous(name = "BlueLeft", group = "Taus")
public class BlueLeft extends AutonomousMethods {

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
                forward(.5, 2, 11);
                shoot(-2, (2244.2*28)/60.0);
                toAngle(60, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(0, 1);
                break;
            case 1:
                //code
                forward(.5, 2, 16);
                shoot(0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 12);
                toAngle(-180, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 2);
                forward(.5, 2, 12);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(.5, 0, 12);
                forward(.5, 0, 9);
                toAngle(0, 1);
                forward(.5, 0, 23);
                shoot(0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 11);
                break;
            case 4:
                //code
                forward(.5, 2, 16);
                shoot(0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 20);
                toAngle(60, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 2);
                toAngle(0, 1);
                strafeLeft(.5, 0, 4);
                backward(.5, 2, 18);
                toAngle(-90, 1);
                backward(.5, 0, 8);
                setIntakePower(1);
                backward(.5, 0, 2);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 4);
                shoot(0, (2360.0*28)/60.0);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(.5, 0, 8);
                forward(.5, 0, 12);
                toAngle(0, 1);
                forward(.5, 0, 23);
                shoot(2.0, (2216.0*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
        }

    }
}
