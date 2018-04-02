#include <iostream>
#include <string>

#include "WPILib.h"
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>


#include "utils.hpp" // tate's collection of robot utilities


class Robot : public frc::IterativeRobot {
public:



	// we have a 6-cim tank drive

	// left side of drivetrain
	frc::PWMVictorSPX left0Mot{0}, left1Mot{1}, left2Mot{2};
	frc::SpeedControllerGroup lMots{ left0Mot, left1Mot, left2Mot };

	// right side of drivetrain
	frc::PWMVictorSPX right0Mot{3}, right1Mot{4}, right2Mot{5};
	frc::SpeedControllerGroup rMots{ right0Mot, right1Mot, right2Mot };

	// drivetrain object
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
	const std::string kAuto = "straight";	 // drive forwards
	const std::string kAutoLeft = "left";   // starting on left side of field
	const std::string kAutoRight = "right"; // starting on right side of field
	const std::string kAutoRightHook = "right hook"; // Right side L pattern
	const std::string kAutoCenter = "center"; // left hook
	std::string m_autoSelected;


	// run when robot boots up
	void RobotInit(){

		std::cout <<"initializing...";
		gyro.Calibrate();

		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAuto,kAuto);
		m_chooser.AddObject(kAutoLeft,kAutoLeft);
		m_chooser.AddObject(kAutoRight, kAutoRight);
		m_chooser.AddObject(kAutoRightHook, kAutoRightHook);
		m_chooser.AddObject(kAutoCenter, kAutoCenter);

		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


		// refresh rate
		drive.SetExpiration(0.1);



		// post USB camera feed to the SmartDashboard
		cs::UsbCamera winchCam = CameraServer::GetInstance()->StartAutomaticCapture(0);
		winchCam.SetResolution(320, 240);


		std::cout <<"done\n";

	}


	// is the left side of ally switch our color?
	bool startLeft(){
		std::cout <<"checking if starting left...";

		// attempt to get gamedata
		// should be in form of "LRL", or something like that (from our perspective)
		std::string msg = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		// keep attempting until we get it
		// will infinite loop on failure
		while (msg.length() == 0) {
			msg = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		}

		std::cout <<"done\n" << "received " << msg <<std::endl;

		// return result of this condition
		return msg[0] == 'L';

	}



	void AutonomousInit() override {

		// get chosen auto
		m_autoSelected = m_chooser.GetSelected();
		std::cout << "Auto selected: " << m_autoSelected <<std::endl;


		// enable motor controllers
		drive.SetSafetyEnabled(false);

		// reset gyro to zero, (not required by functions, but still good habit
		//gyro.Reset();
		std::cout <<"gyro reset to " <<gyro.GetAngle() <<" degrees\n";

		// drive straight dont dump
		if (m_autoSelected == kAuto) {
			utils::driveStraight(gyro, drive, 4, -0.5);

		// left hook auto
		} else if (m_autoSelected == kAutoLeft) {
			bool autoErrors = false;

			autoErrors |= utils::driveStraight(gyro, drive, 4.5, -0.5, this); // err = err || drive();
			autoErrors |= utils::turnDeg(gyro, drive, 90, this);
			autoErrors |= utils::driveStraight(gyro, drive, 2, -0.5, this);


			if(startLeft() && !autoErrors) {
				flipper.Set(frc::DoubleSolenoid::kForward);
			}

		// right auto
		} else if (m_autoSelected == kAutoRight) {
			bool autoErrors = utils::driveStraight(gyro, drive, 4, -0.5, this);

			// if everything executed flawlessly and we're on the correct side
			if(!startLeft() && !autoErrors) {
				flipper.Set(frc::DoubleSolenoid::kForward);
			}

		// right auto
		} else if (m_autoSelected == kAutoRightHook) {
			bool autoErrors = false;
			autoErrors |= utils::driveStraight(gyro, drive, 4.5, -0.5, this);
			autoErrors |= utils::turnDeg(gyro, drive, -90, this);
			autoErrors |= utils::driveStraight(gyro, drive, 2, -0.5, this);


			// if everything executed flawlessly and we're on the correct side
			if(!startLeft() && !autoErrors) {
				flipper.Set(frc::DoubleSolenoid::kForward);
			}

		// center auto, Y pattern
		} else if (m_autoSelected == kAutoCenter) {

			bool autoErrors = utils::driveStraight(gyro, drive, 1.5, -0.5, this); // drive forward a bit

			// branch toward correct side of switch
			if (startLeft()) {
				autoErrors |= utils::turnDeg(gyro, drive, -90, this);		// turn left
				autoErrors |= utils::driveStraight(gyro, drive, 2, -0.5, this);	// drive forward
				autoErrors |= utils::turnDeg(gyro, drive, 90, this);		// turn right
				autoErrors |= utils::driveStraight(gyro, drive, 2, -0.5, this);	// drive straight

			} else {
				autoErrors |= utils::turnDeg(gyro, drive, 90, this);		// turn right
				autoErrors |= utils::driveStraight(gyro, drive, 2, -0.5, this);	// drive straight
				autoErrors |= utils::turnDeg(gyro, drive, -90, this);		// turn left
				autoErrors |= utils::driveStraight(gyro, drive, 2, -0.5, this);	// drive straight

			}

			// if everything executed flawlessly then dump
			if (!autoErrors) {
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
		* b - slow-mode (hold for slow)
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



		// utils::expReduceBrownout averages in previous driving inputs to
		// make driving smoother. inputs will be stored here
		static double avgx = 0.0, avgy = 0.0;


		// arcade drive with 2 sticks & 80% turn speed
		// slow-mode controlled by driver (button B)
		drive.ArcadeDrive(

			utils::expReduceBrownout(
				// forward backwards
				(driveCtl.GetRawButton(2) ? 0.4 : 1.0) * // 40% movespeed in slowmode
					dir * // +/- 1 (forward/reverse)
						driveCtl.GetRawAxis(1)
				, avgy
			)
			,
			utils::expReduceBrownout(
				// left right
				(driveCtl.GetRawButton(2) ? 0.8 : 1.0) * // .8*.8 = 64% turnspeed in slowmode
					driveCtl.GetRawAxis(4) * 0.8
				, avgx
			)
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
		armCtl.Set(fxnCtl.GetRawAxis(1) * -0.6);


		// control rollers
		if (fxnCtl.GetRawButton(4)) {
			lRoller.Set(.75);
			rRoller.Set(.75);

		} else if (fxnCtl.GetRawButton(3)) {
			lRoller.Set(-1);
			rRoller.Set(-1);

		} else {
			lRoller.Set(0);
			rRoller.Set(0);
		}

	}

	void TestPeriodic() {}

};


START_ROBOT_CLASS(Robot);
