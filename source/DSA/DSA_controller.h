#ifndef DSA_CONTROLLER_H
#define DSA_CONTROLLER_H

#include <source/Base/BaseController.h>
#include <source/DSA/DSA_loop_functions.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <cmath>

using namespace argos;
using namespace std;

class DSA_loop_functions;

static unsigned int num_targets_collected = 0;

class DSA_controller : public BaseController {

    public:

        DSA_controller();

        // CCI_Controller inheritence functions
        void Init(TConfigurationNode& node);
        void ControlStep();
        void Reset();

        bool   IsHoldingFood();
        bool   IsInTheNest(); //qilu 02/2023
        void   getSpiralPath();
        void   SetRobotPath(string path);
		
		void   writePatternToFile(vector<char>&, int N_robots);
		void   addDirectionToPattern(char direction);
		void   printPath(vector<char>&);

		void SetLoopFunctions(DSA_loop_functions* lf) { loopFunctions = lf; }

		argos::Real SimTimeInSeconds();

    private:
  string 			controllerID;
		size_t	RobotID; // start from 0 qilu 12/2022

        size_t NumberOfRobots;
        size_t NumberOfSpirals;

        /* Robot DSA state variable */
        enum DSA { START = 0, SEARCHING = 1, RETURN_TO_NEST = 2, RETURN_TO_SEARCH = 3, IDLE = 4 } DSA;

        /* robot internal variables & statistics */
        CRandom::CRNG*      RNG;
        DSA_loop_functions* loopFunctions;

        CVector2            ReturnPosition;
        CVector2            ReturnSpiralPosition;

        vector<CRay3>       myTrail;
        CColor              TrailColor;

	Real                ProbTargetDetection;
        Real                SearcherGap;
        Real                FoodDistanceTolerance;
        Real                SquaredFoodDistanceTolerance;
       	CVector2            previous_position;
	CVector2            previous_target;
	CVector2            newTarget;
        CVector3            startPosition;
        vector<char>        pattern;
        vector<char>        tempPattern;
        vector<string>      rPattern;
        int                 levels;
        bool                isHoldingFood;
        bool                goingHome;
        bool                ResetReturnPosition;
        CRange<CRadians>    AngleToleranceInRadians;
        CRange<CRadians>    Tolerance;
        size_t              stopTimeStep;
        size_t              collisionDelay;
	char direction_last;

        /* movement functions */
        CDegrees angleInDegrees;

        void SetTargetN(char x);
        void SetTargetS(char x);
        void SetTargetE(char x);
        void SetTargetW(char x);
    
        /* movement helper functions */
        void ReachSpiralTargets();
        void CopyPatterntoTemp();
        bool TargetHit();
        void SetHoldingFood(); 

	string results_path;
	string results_full_path;
		/* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;
};

#endif /* DSA_CONTROLLER_H */
