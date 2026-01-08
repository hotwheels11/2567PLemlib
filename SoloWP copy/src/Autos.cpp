#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: keep
#include "ExternalSystems.hpp" // IWYU pragma: keep

void line_Up_Auto(){
    chassis.moveToPoint(0,14,2000,{.maxSpeed = 40});
}
void line_Up_Solo(){
    chassis.setPose(-62.469,-16.3,90);
	chassis.moveToPose(-47,-16.3,90,2000,{.maxSpeed=35});
}

void WP_Auto(){
    
    chassis.setPose(-47,-16.3,90);
	intake.move(127);
	chassis.moveToPoint(-33.5,-16.3,800);
	chassis.waitUntilDone();
	chassis.swingToHeading(134, lemlib::DriveSide::RIGHT, 200,{},false);
	chassis.waitUntil(2.7);
	intakemiddle.move(50);
	Matchload.set_value(1);
	chassis.moveToPoint(-16.7,-23,800);
	chassis.waitUntilDone();
	intakemiddle.move(0);
	chassis.turnToHeading(49, 600);
	chassis.waitUntilDone();
	Matchload.set_value(0);
	chassis.moveToPoint(-12.3, -19, 800);
	chassis.waitUntilDone();
	intake.move(-127);
	pros::delay(750);
	intake.move(127);
	chassis.moveToPoint(-17, -27.6, 800,{.forwards=false});
	chassis.waitUntilDone();
	chassis.turnToHeading(0, 400);
	chassis.waitUntilDone();
	chassis.moveToPoint(-15, 18, 1200,{.maxSpeed=100});
	chassis.waitUntil(38);
	Matchload.set_value(1);
	chassis.waitUntilDone();
	chassis.turnToHeading(-44, 400);
	chassis.waitUntilDone();
	chassis.moveToPoint(-1, 6,800,{.forwards=false});
	chassis.waitUntil(2);
	Matchload.set_value(0);
	chassis.waitUntilDone();
	intake.move(-127);
	pros::delay(400);
	intake.move(127);
	intakelate.move(-127);
	pros::delay(800);
	intakelate.move(127);
	chassis.turnToHeading(-38, 700,{},false);
	chassis.moveToPoint(-38, 46, 1200,{.maxSpeed=90});
	chassis.waitUntilDone();
	chassis.turnToHeading(-90, 600);
	chassis.waitUntilDone();
	Matchload.set_value(1);
	pros::delay(50);
	chassis.moveToPoint(-70, 46, 1500,{.maxSpeed=60});
	chassis.moveToPoint(-14, 47, 3000,{.forwards=false,.maxSpeed=60});
	chassis.waitUntil(4);
	intake.move(0);
	pros::delay(500);
	Hood.set_value(1);
	intake.move(127);
}

void right_Side() {
    chassis.setPose(0, 0, 0);
	Descore.set_value(1);
    chassis.moveToPoint(0, 13, 1000);
    intake.move(127);
    chassis.moveToPoint(9.0, 29, 1000);//get first three balls
    chassis.waitUntil(3);
    Matchload.set_value(1); //Activates to hold balls in place
    chassis.waitUntilDone();
    Matchload.set_value(0); //Deactivates
    pros::delay(1000);
    chassis.moveToPoint(26.5, 48, 2000);
    pros::delay(500);
    Matchload.set_value(1); //Activates to hold balls in place
    chassis.moveToPoint(9,29,1000,{.forwards=false,.maxSpeed=60}); //moves back to three ball area
    chassis.turnToHeading(-41.5,500); //moves to middle goal
    Matchload.set_value(0); //Deactivates
    chassis.moveToPoint(3, 41, 800); //drives into middle goal
    intake.move(-127); //outtakes balls into middle goal
    pros::delay(1350);
    intake.move(127);//starts intake fwd
    intakeFront.move(40);
    chassis.moveToPoint(37, 6.7, 1300,{.forwards=false,.maxSpeed=80}); //backs away from middle goal
    chassis.turnToHeading(-180,1000);
    Matchload.set_value(1); //Activates
    chassis.moveToPoint(33.5,-10,1600,{.maxSpeed=60}); //Drives into Matchload Area
    chassis.waitUntilDone();
    intakeFront.move(127);
    chassis.moveToPoint(36, 30, 1000,{.forwards=false,.maxSpeed=70}); //backs out of matchload area
    chassis.waitUntilDone();
    intake.move(-127);
    pros::delay(200);
    intake.move(127);
    Hood.set_value(1);
    intake.move(127);
    Matchload.set_value(0); //Deactivates
    Descore.set_value(0);
}

void left_Side() {
    chassis.setPose(0, 0, 0);
	Descore.set_value(1);
    chassis.moveToPoint(0, 13, 700);
    intake.move(127);
    chassis.moveToPoint(-9.0, 28, 700);//get first three balls
    chassis.waitUntil(3.5);
    Matchload.set_value(1); //Activates to hold balls in place
    chassis.waitUntilDone();
    Matchload.set_value(0); //Deactivates
    pros::delay(400);
	chassis.turnToHeading(-43, 800);
    chassis.moveToPoint(-24, 45.5, 1500);
    pros::delay(450);
    Matchload.set_value(1); //Activates to hold balls in place
    chassis.moveToPoint(-7,26.4,1000,{.forwards=false,.maxSpeed=50}); //moves back to three ball area
    chassis.turnToHeading(-133, 1000); //turn to middle goal
    Matchload.set_value(0); //Deactivates
    chassis.moveToPoint(12, 47, 800,{.forwards=false}); //drives into middle goal
	intake.move(-127);
	intakeFront.move(0);
	pros::delay(920);
    intake.move(127); //outtakes balls into middle goal
	intakelate.move(-70);
    pros::delay(1000);
	intakelate.move(127);
    chassis.moveToPoint(-30, 9.9, 1300,{.maxSpeed=90}); //backs away from middle goal
    chassis.turnToHeading(-180,1000,{.maxSpeed=90});
    Matchload.set_value(1); //Activates
    chassis.moveToPoint(-32.1,-14,1600,{.maxSpeed=50}); //Drives into Matchload Area
    chassis.waitUntilDone();
    intakeFront.move(127);
    chassis.moveToPoint(-32.1, 38, 2000,{.forwards=false,.maxSpeed=60}); //backs out of matchload area
    chassis.waitUntilDone();
    Hood.set_value(1);
    Matchload.set_value(0); //Deactivates
    intake.move(-127);
    pros::delay(300);
    intake.move(127);
    /*
    pros::delay(800);
    chassis.moveToPoint(-32.2,30,1000);
    chassis.waitUntilDone();
    Descore.set_value(0);
    chassis.moveToPoint(-32.2, 38, 2000,{.forwards=false,.maxSpeed=60}); //backs out of matchload area
    chassis.waitUntilDone();
*/
}

void calculatePoseAuto() {
    Matchload.set_value(1); //Activates to hold balls in place
    pros::delay(1000);
    calculates_robot_position(); // saves automatically in savedPose
    pros::delay(1000);
    Matchload.set_value(0); //Deactivates
}

void Skills_Auto(){
    setChassisToSavedPose(); // sets the chassis to previously saved position
    Descore.set_value(1);
    chassis.moveToPoint(46,43.5,1000,{.maxSpeed=80}); //Goes fwd to get perpendiuclar for matchload #1
    chassis.waitUntilDone();
    chassis.turnToHeading(90,700); // Turns towards Matchload #1
    chassis.waitUntilDone();
    pros::delay(300); //Allows time for matchload to Deploy
    Matchload.set_value(1);//Deploys Matchload
    chassis.moveToPoint(60,50.5,3000,{.maxSpeed=50}); //Driveinto matchloadpoing while intake (timeoutcontrolls the length of intake)
    intake.move(127);
    chassis.waitUntilDone();
    Matchload.set_value(0); //PicksUp Matchload
    chassis.moveToPoint(49.1,50,800,{.forwards=false});//Reverses out of Matchload #1
    chassis.waitUntilDone();
    chassis.turnToHeading(-40,700); //Turns towards the alley way
    chassis.waitUntilDone();
    chassis.moveToPoint(30,62.5,1000,{.maxSpeed=100}); // Attempts to get next to the long goal support
    chassis.waitUntil(12); //waits until 12 inches has passed
    chassis.cancelMotion(); //Stops attempting to go to previous point
    chassis.moveToPoint(-25,61,2000,{.maxSpeed=100}); //Attempts to go to the second point next to the opposite end of long goal
    chassis.waitUntil(60);//Waits until 12 inches has passed
    chassis.cancelMotion();//stops trying to go to previous point
    chassis.moveToPoint(-45,48,1500,{.maxSpeed=80}); //Gets infront of long goal
    chassis.waitUntilDone();
    chassis.turnToHeading(-90,900); //Turn the back to the goal
    chassis.waitUntilDone();
    chassis.moveToPoint(-20,47,3100,{.forwards=false,.maxSpeed=60}); //reverses into the goal while allowing the intake to run at the same time
    intakemiddle.move(-127);
    pros::delay(400);
    intake.move(127);
    chassis.waitUntil(11.5);//waits until 11.5 inches has passed
    intake.move(127);
    Hood.set_value(1); //Lifts Hood
    chassis.waitUntilDone();
    Matchload.set_value(1);//Deploys Matchload
    chassis.moveToPoint(-65,47,4000,{.maxSpeed=45}); //Goes into matchload while intaking
    pros::delay(300);
    Hood.set_value(0); //Lowers Hood
    chassis.waitUntilDone();
    chassis.moveToPoint(-20,47,4000,{.forwards=false,.maxSpeed=60}); //backs into long goal again
    chassis.waitUntil(28); //waits until 15 inches passed
    intake.move(-127);
    Hood.set_value(1); // Lifts Hood
    pros::delay(300);
    intake.move(127);
    chassis.waitUntilDone();
    chassis.moveToPoint(-40,46,1000); //Pulls out of long goal
    chassis.waitUntilDone();
    Hood.set_value(0);
    chassis.moveToPoint(-20,46,2000,{.forwards=false,.maxSpeed=25}); //Lightly pushes blocks into middle area
    chassis.waitUntilDone();
    chassis.moveToPoint(-49.6,36,1000);//starts a cruve turn
    chassis.waitUntil(16);
    chassis.cancelMotion();
    chassis.moveToPoint(-38,0,1000);//goes to infront of the blue area
    chassis.waitUntil(25);
    chassis.cancelMotion();
    chassis.moveToPoint(-51.4,-49,2000,{.maxSpeed=80}); //perpendicular to matchload #3
    chassis.waitUntilDone();
    chassis.turnToHeading(-90,700); //Turns to Matchload #3
    chassis.waitUntilDone();
    Matchload.set_value(1);
    chassis.moveToPoint(-66,-56,4000,{.maxSpeed=55}); //Goes into matchload #3
    chassis.waitUntilDone();
    chassis.moveToPoint(-49,-58.5,1000,{.forwards=false,.maxSpeed=60}); //Pulls out of Matchload #3
    chassis.waitUntilDone();
    Matchload.set_value(0);
    chassis.turnToHeading(-220,700); //Turns to the allyway
    chassis.waitUntilDone();
    chassis.moveToPoint(-28,-68,1000,{.maxSpeed=80});//goes into the alleyway
    chassis.waitUntilDone();
    chassis.turnToHeading(-270,700);
    chassis.moveToPoint(24,-67,2000,{.maxSpeed=80}); //gets ready to exit alley way
    chassis.waitUntilDone();
    chassis.moveToPoint(40,-59,1000); //gets infront of long goal #2
    chassis.waitUntilDone();
    chassis.turnToHeading(-270,700); //Turns to long goal #2
    chassis.waitUntilDone();
    chassis.moveToPoint(20,-52,3500,{.forwards=false,.maxSpeed=60}); //reverses into long goal #2
    chassis.waitUntil(15);
    intake.move(-127);
    Hood.set_value(1);
    pros::delay(300);
    intake.move(127);
    chassis.waitUntilDone();
    Matchload.set_value(1);
    chassis.moveToPoint(60,-53.5,4000,{true,50});//goes into matchload #4
    pros::delay(200);
    Hood.set_value(0);
    chassis.waitUntilDone();
    Matchload.set_value(0);
    chassis.moveToPoint(18,-54,4000,{false,60}); //backs out of matchload #4 and goes into long goal #2
    chassis.waitUntil(23);
    intake.move(127);
    Hood.set_value(1);
    chassis.waitUntilDone();
    chassis.moveToPoint(30,-54,1000);
    chassis.waitUntilDone();
    Hood.set_value(0);
    chassis.moveToPoint(18,-54,1500,{false,30}); //backs out of matchload #4 and goes into long goal #2
    chassis.waitUntilDone();
    chassis.moveToPoint(50,-48,1000); //gets ready to go into red zone
    chassis.waitUntilDone();
    chassis.moveToPoint(63,-20,1000); //final line up
    chassis.waitUntilDone();
    Matchload.set_value(1);
    pros::delay(1000);
    chassis.moveToPoint(63,30,2000); //goes into redzone
    chassis.waitUntilDone();
    Matchload.set_value(0);
} 