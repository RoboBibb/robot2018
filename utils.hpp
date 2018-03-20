#ifndef FRC4941_UTILS_HPP
#define FRC4941_UTILS_HPP

#include "WPILib.h"
#include <cmath> // cos, pow, abs




/* inline lesson in C++
 * so new people remember how things work next year
 * ternary operators:
 *  - shorthand if
 *  - ` condition ? when true : when false;
 *
 * namespaces
 * 	- accessed using scope operator `::`
 * 	```
 * 	namespace ns {
 * 		const int num = 5;
 * 	}
 *
 * 	std::cout <<ns::num;
 * 	```
 *
 * fxn inlining:
 * 	- compiler optimization for short functions
 * 	- Completely optional
 *
 * function pointers:
 *  - all functions are technically pointers
 *  - we used them to pass a function as an argument to a function
 *
 */

/// a few useful functions for the robot
namespace utils {


	/// remove 'ghost-input' resulting from inaccurate joysticks
	// input < +/- 15% gets ignored
	inline double removeGhost(const double val)
		{ return (val > 0.05 || val < -0.05) ? val : 0.0; }


	/// a linear approach to preventing brownout (untested)
	// prevents input from changing faster than a certain rate
	// could probably be combined with expReduceBrounout to improve effectiveness
	double linReduceBrownout(const double limit, const double current, double& past)
	{
		/// limit = maximum amount of change per cycle
		/// current = the most recent value coming from input
		/// past = the value returned by this function in the last frame

		// null or ghost input doesn't affect robot (also good for breaking)
		if (utils::removeGhost(current) == 0.0f) return 0.0;

		double change = current - past;

		if (change > 0) { // increase speed
			if (change > limit) // too much change
				return (past += limit);
			// nominal change
			return (past = current);

		} else { // decrease speed
			if (change < -limit) // too much change
				return (past -= limit);
			// nominal change
			return (past = current);
		}
	}

	/// plots input on a curve to make driving different
	inline double unsignedSqrt(const double val)
		{ return val > 0 ? sqrt(val) : -sqrt(-val); }

	/// plots input on a curve to make driving different
	inline double unsignedPow2(const double val)
		{ return val > 0 ? val * val : -val * val; } // change sign to match input

	/// a nice function to use for drive curves
	inline double huntFunt(const double val)
		{ return val > 0 ? -cos(val * M_PI / 2) + 1 : -(-cos(val * M_PI / 2) + 1); }


	/// averages in the previous value to make the change less drastic
	// TODO: try using a cube-root/cubic/x^2 curve
	inline double expReduceBrownout(const double current, double& past)
		{ return unsignedPow2( past = (past + current) / 2 ); }

	// this way we can use fxns other than unsignedPow2
	inline double expReduceBrownout(const double current, double& past, double (*inputCurve)(double))
		{ return inputCurve( past = (past + current) / 2 ); }

	// recycled from last year's utils.hpp
	// drives straight for a set period of time at a set speed
	bool driveStraight(frc::ADXRS450_Gyro& gyro, frc::DifferentialDrive& mots, const double time, const double speed = 0.5){

		std::cout <<"Driving straight for " <<time <<"seconds at " <<speed <<"power... ";
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
		for (int i = (int) (time / cycletime); i > 0; i--) {
			// turn to correct heading
			mots.ArcadeDrive(speed, gyro.GetAngle() * turningConst); // add negatives for inverted steering/drive
			// drive straight a bit before readjusting steering
			Wait(cycletime);
		}

		mots.ArcadeDrive(0.0, 0.0);

		std::cout <<"done (dif="<<gyro.GetAngle() <<")\n";

		// did we fail to keep straight? (5 deg tolerance)
		return std::abs(gyro.GetAngle()) > 5;

	}



	/// this will turn the robot a set number of degrees (can be negative)
	// note, this isn't accurate as we aren't using calculus and a feedback loop, but should be good enough
	// IsAutonomous points to RobotBase::IsAutonomous() so we can avoid infinite loops which prevent us from ending auto
	bool turnDeg(frc::ADXRS450_Gyro& gyro, frc::DifferentialDrive& mots, double angleDeg, RobotBase* robot = nullptr){

		const double cycleTime = 0.004, // prevent cpu taxing
					 tolerance = 5,		// how close is "good enough" (values too low cause infinite spinning)
					 turnSpeed = 0.5;


		gyro.Reset();

		std::cout <<"turning " <<angleDeg <<" degrees... ";

		// turning right
		if (angleDeg > 0) {
			while (gyro.GetAngle() - angleDeg < -tolerance && robot ? robot->IsAutonomous() : false) {
				// turn at 50% speed until u get within 10 deg,
				// then lower speed linearly as you approach
				mots.ArcadeDrive(0,
						std::abs(angleDeg - gyro.GetAngle()) > 10 ?
							turnSpeed :
							turnSpeed * (gyro.GetAngle() - angleDeg) / 10
								+ 0.1 * angleDeg - gyro.GetAngle > 0 ? 1 : -1
				);

				// prevent CPU taxing
				Wait(cycleTime);
			}

		// turning left
		} else {
			while (gyro.GetAngle() - angleDeg > tolerance && robot ? robot->IsAutonomous() : false) {
				// turn at 50% speed until u get within 10 deg,
				// then lower speed linearly as you approach
				mots.ArcadeDrive(0,
						-(std::abs(angleDeg - gyro.GetAngle()) > 10 ?
								turnSpeed :
								turnSpeed * -(angleDeg - gyro.GetAngle()) / 10
									+ 0.1 * angleDeg - gyro.GetAngle > 0 ? 1 : -1)
				);

				// prevent CPU taxing
				Wait(cycleTime);
			}

		}

		mots.ArcadeDrive(0.0, 0.0);

		std::cout <<"done (dif = " <<gyro.GetAngle() - angleDeg <<")\n";

		// did we fail to turn the correct angle (5 deg tolerance)
		return std::abs(gyro.GetAngle() - angleDeg) > 5;
	}

}




#endif
