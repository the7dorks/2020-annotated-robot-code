/**
 * Trajectory.cpp
 *
 * This file contains the definitions of the Trajectory class. The Trajectory class reads and stores
 * motion profile information from the sd card. Motion profiles stored on the sd card are calculated
 * by the publically availible GitHub repository, TrajectoryLib by Team254 (FRC Team 254, The Cheesy
 * Poofs), found here: https://github.com/Team254/TrajectoryLib. The trajectories are calculated on
 * a computer, and stored on the sd card for the robot to use. Each time step of the profile is read
 * from the sd card, and stored in an instance of ProfileStep by the Trajectory class.
 *
 * The paths are intended to be executed by the Drivetrain class, but are not used in programming
 * skills.
 */
#include "main.h" // gives access to Trajectory and other dependencies

/* ----------------------------------------------------------- */
/*                      Public Information                     */
/* ----------------------------------------------------------- */
Trajectory::Trajectory(const char * ifileName, double ikP, double ikD, double ikV, double ikA)
    : mkP(ikP), mkD(ikD), mkV(ikV), mkA(ikA), mstepNumber(0), mlastErrorL(0.0),
      mlastErrorR(0.0) // constructor that specifies the file with the trajectory, and the gains for
                       // following the trajectory
{
    FILE * file; // creates a file object to be used later
    if (pros::usd::is_installed()) // checks if the sd card is installed before trying to access it
    {
        file = fopen(ifileName, "r"); // open the file
        if (file) // makes sure the file was opened correctly
        {
            char name[256];
            fgets(name, 255, file); // put the name of the trajectory in a char array
            mname = name;
            mname = mname.substr(0, mname.length() - 1); // chop off \n

            fscanf(file, "%i", &mlength); // store the number of steps

            mleftSteps = new ProfileStep[mlength]; // dynamically allocate left and right profiles
                                                   // based on the length of the trajectory
            mrightSteps = new ProfileStep[mlength];

            for (int i = 0; i < mlength; i++) // fill left profile array from sd card
            {
                float pos, vel, acc, jerk, heading, dt, x, y;
                fscanf(file, "%f %f %f %f %f %f %f %f", &pos, &vel, &acc, &jerk, &heading, &dt, &x,
                       &y);
                mleftSteps[i] = {pos, vel, acc, jerk, heading, dt, x, y};
            }
            for (int i = 0; i < mlength; i++) // fill right profile array from sd card
            {
                float pos, vel, acc, jerk, heading, dt, x, y;
                fscanf(file, "%f %f %f %f %f %f %f %f", &pos, &vel, &acc, &jerk, &heading, &dt, &x,
                       &y);
                mrightSteps[i] = {pos, vel, acc, jerk, heading, dt, x, y};
            }
        }
        else
        {
            std::cout << "\"" << ifileName << "\" file is null"
                      << std::endl; // output to the terminal if the file is null
        }

        fclose(file);
    }
    else // if the sd card is not installed, create empty arrays and send a message to the terminal
    {
        mleftSteps = new ProfileStep[1];
        mrightSteps = new ProfileStep[1];
        std::cout << "no sd card inserted" << std::endl;
    }
}
Trajectory::~Trajectory() // destructor to clean up heap variables
{
    delete[] mleftSteps;
    delete[] mrightSteps;
}

int Trajectory::getLength() { return mlength; }
std::string Trajectory::getName() { return mname; }
void Trajectory::reset() { mstepNumber = 0; }
std::pair<ProfileStep, ProfileStep>
Trajectory::getStep(int istepNumber) // return the left and right ProfileSteps at a given point
{
    if (istepNumber >
        sizeof(mleftSteps) / sizeof(ProfileStep *)) // if the stepnumber is out of range
    {
        std::cout << "index out of bounds" << std::endl;
    }
    return std::pair<ProfileStep, ProfileStep>(mleftSteps[istepNumber], mrightSteps[istepNumber]);
}
void Trajectory::setGains(const double ikP, const double ikD, const double ikV, const double ikA)
{
    mkP = ikP;
    mkD = ikD;
    mkV = ikV;
    mkA = ikA;
}
bool Trajectory::isDone() // checks to see if the trajectory is done
{
    return mlength <= mstepNumber;
}

std::pair<double, double>
Trajectory::iterate(double ileftDistSoFar,
                    double irightDistSoFar) // goes through one iteration of the FEEDFORWARD loop
                                            // based on how far the left and right wheels have gone
{
    if (mstepNumber < mlength)
    {
        double errorL = mleftSteps[mstepNumber].pos -
                        ileftDistSoFar; // diference between where the wheels should be, and where
                                        // they are for feedback control
        double errorR = mrightSteps[mstepNumber].pos - irightDistSoFar;
        double errorVelL = ((errorL - mlastErrorL) / mleftSteps[mstepNumber].dt -
                            mleftSteps[mstepNumber].vel); // velocity error
        double errorVelR =
            ((errorR - mlastErrorR) / mrightSteps[mstepNumber].dt - mrightSteps[mstepNumber].vel);

        double resultL =
            mkP * errorL + mkD * errorVelL + mkV * mleftSteps[mstepNumber].vel +
            mkA * mleftSteps[mstepNumber].acc; // Kp*ep(t) + Kd*ev(t) + Kv*rv(t) + Ka*ra(t)
        double resultR = mkP * errorR + mkD * errorVelR + mkV * mrightSteps[mstepNumber].vel +
                         mkA * mrightSteps[mstepNumber].acc;

        mlastErrorL = errorL; // store error for derivative calculation next time
        mlastErrorR = errorR;

        mstepNumber++;

        return {resultL, resultR}; // return the necessary motor movements
    }
    else
    {
        return {0, 0}; // return 0 power for both motors, because the path has finished executing
    }
}