#include <iostream>
#include <string>

#include "WPILib.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

class Robot : public frc::IterativeRobot {
public:



	// we have a 6-cim tank drive 


	// left side of drivetrain
	frc::PWMVictorSPX left0Mot{0}, left1Mot{1}, left2Mot{2};
	frc::SpeedControllerGroup lMots{ left0Mot, left1Mot, left2Mot };

	// right side of drivetrain
	frc::PWMVictorSPX right0Mot{3}, right1Mot{4}, right2Mot{5};
	frc::SpeedControllerGroup rMots{ right0Mot, right1Mot, right2Mot };

	// drive train object
	frc::DifferentialDrive drive{lMots, rMots};

	// raising and lowering the arms
	frc::Talon armCtl{7};

	// roller arms (intake/shoot)
	frc::Spark lRoller{8};
	frc::Spark rRoller{9};

	// claw to grab boxes
	frc::DoubleSolenoid grabber{0, 1};
	// launch cube from front
	frc::DoubleSolenoid flipper{7, 6};



	// main gyro (SPI port - plugs to rio)
	frc::ADXRS450_Gyro gyro;

	// xbox ctlr for main driver
	frc::Joystick driveCtl{0};
	frc::Joystick fxnCtl{1};



	// autonomous chooser on DS
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance(); // this needed?
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoDriveStraight = "straight";	 // drive forwards
	const std::string kAutoDriveStraightLeft = "left";   // starting on left side of field
	const std::string kAutoDriveStraightRight = "right"; // starting on right side of field
	std::string m_autoSelected;


	// made in despiration for converting a number to a string, trying to make auto
	//  chooser working...
	const std::string autos[4] = { kAutoNameDefault,
		kAutoDriveStraight,
		kAutoDriveStraightLeft,
		kAutoDriveStraightRight
	};

	// run when robot boots up
	void RobotInit(){

		std::cout <<"initializing...";
		gyro.Calibrate();

		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoDriveStraight,kAutoDriveStraight);
		m_chooser.AddObject(kAutoDriveStraightLeft,kAutoDriveStraightLeft);
		m_chooser.AddObject(kAutoDriveStraightRight, kAutoDriveStraightRight);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

		// newlines dont work...
		frc::SmartDashboard::PutNumber("auto mode\n"
				"<0> default (do nothing)\n"
				"<1> drive straight (no dump)\n"
				"<2> drive straight (left)\n"
				"<3> drive straight (right)", 0);

		// refresh rate
		drive.SetExpiration(0.1);

		std::cout <<"done\n";

	}





	// recycled from last year's utils.hpp
	// drives straight for a set period of time at a set speed
	void driveStraight(frc::ADXRS450_Gyro& gyro, frc::DifferentialDrive& mots, const double time, const double speed = 0.5){

		// did some math to guesstimate these values
		const double 
			turningConst = -0.03, // if it doesnt work negate this
			cycletime = 0.004;

		// get angle to maintain as zero
		gyro.Reset();

		// drive forward for the set ammount of time
		// cycletime is determined based on the speed of the robot
		//		slower speed = longer input cycles
		//		faster speed = shorter input cycles
		for (int i = (int) (time / (cycletime / abs(speed))); i > 0; i--) {
			// turn to correct heading
			mots.ArcadeDrive(speed, gyro.GetAngle() * turningConst); // add negatives for inverted steering/drive
			// drive straight a bit before readjusting steering
			Wait(cycletime / abs(speed));
		}

		mots.ArcadeDrive(0.0, 0.0);

	}

	// is the left side of ally switch our color?
	bool startLeft(){
		std::cout <<"checking if starting left...";

		// attempt to get gamedata
		// should be in form of "LRL", or something like that (from our perspective)
		std::string msg = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		// keep attempting until we get it
		while (msg.length() == 0) {
			msg = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		}

		std::cout <<"done\n";

		// return result of this condition
		return msg[0] == 'L';

	}



	void AutonomousInit() override {

		// get chosen auto
		m_autoSelected = m_chooser.GetSelected();
		std::cout << "Auto selected: " << m_autoSelected <<std::endl; // doesn't work (empty string)

		m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected <<std::endl; // doesn't work("default")



		// worked twice then stopped letting us change the value
		m_autoSelected = autos[(int)
			SmartDashboard::GetNumber("auto mode\n"
				"0 - default (do nothing)\n"
				"1 - drive straight (no dump)\n"
				"2 - drive straight (left)\n"
				"3 - drive straight (right)", 0)];
		std::cout << "Auto selected: " << m_autoSelected <<std::endl; // default


		// enable motor controllers
		drive.SetSafetyEnabled(false);






		// drive straight dont dump
		if (m_autoSelected == kAutoDriveStraight) {
			driveStraight(gyro, drive, 2, -0.5);



		// left auto
		} else if (m_autoSelected == kAutoDriveStraightLeft) {
			driveStraight(gyro, drive, 2, -0.5);
			if(startLeft()) {
				flipper.Set(frc::DoubleSolenoid::kForward);
			}

		// right auto
		} else if (m_autoSelected == kAutoDriveStraightRight) {
			driveStraight(gyro, drive, 2, -0.5);
			if(!startLeft()) {
				flipper.Set(frc::DoubleSolenoid::kForward);
			}


		} else {
			// failsafe, do nothing
		}


	}


	
	void AutonomousPeriodic(){}



	void TeleopInit(){}
	void TeleopPeriodic(){



		/* Driving
		* a - reverse (toggle)
		* b - slowmode (hold for slow)
		* stick1 y - forwards & backwards
		* stick2 x - left & right turning
		*/

		// reverse button is a on drive controller
		// static variables' values are retained in-between cycles
		// this code makes the button "sticky" and toggleable
		static bool switchable = true;
		static int dir = 1;
		if (switchable && driveCtl.GetRawButton(1)) {
			dir = -dir;
			switchable = false;
			std::cout <<"driving reversed\n";
		} else if (!switchable && !driveCtl.GetRawButton(1)) {
			switchable = true;
		}


		// arcade drive with 2 sticks & 80% turn speed
		// slowmode controlled by driver (button B)
		drive.ArcadeDrive(

			// forward backwards
			(driveCtl.GetRawButton(2) ? 0.4 : 1.0) * // 40% movespeed in slowmode
				dir * // +/- 1
					driveCtl.GetRawAxis(1)

			,

			// left right
			(driveCtl.GetRawButton(2) ? 0.8 : 1.0) * // .8*.8 = 64% turnspeed in slowmode
				driveCtl.GetRawAxis(4) * 0.8

		);









		/* functionaity
		* a - close grabber
		* b - open grabber
		* x - intake
		* y - shoot
		* stick1 y - arm elevator
		* right trigger - pop up dumper/flipper
		*/


		// flipper control
		if (fxnCtl.GetRawAxis(3) > 0.75) {
			flipper.Set(frc::DoubleSolenoid::kForward);
		} else {
			flipper.Set(frc::DoubleSolenoid::kReverse);
		}

		// grabber control
		if (fxnCtl.GetRawButton(1)) {
			grabber.Set(frc::DoubleSolenoid::kForward);
		} else if (fxnCtl.GetRawButton(2)) {
			grabber.Set(frc::DoubleSolenoid::kReverse);
		}


		// elevator ctl
		armCtl.Set(fxnCtl.GetRawAxis(1) * -0.4);


		// control rollers
		if (fxnCtl.GetRawButton(4)) {
			lRoller.Set(.75);
			rRoller.Set(.75);
			std::cout <<"Kobe";
		} else if (fxnCtl.GetRawButton(3)) {
			lRoller.Set(-.75);
			rRoller.Set(-.75);
			std::cout <<"Slurp";
		} else {
			lRoller.Set(0);
			rRoller.Set(0);
		}

	}

	void TestPeriodic() {}

};

START_ROBOT_CLASS(Robot)
