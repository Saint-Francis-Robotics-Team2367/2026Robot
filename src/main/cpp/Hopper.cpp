#include "Hopper.h"

void Hopper::init() {
    hopperMotor.GetConfigurator().Apply(hopperConfig);
    
    hopperConfig.Slot0.kP = HopperConstants::hopperP;
    hopperConfig.Slot0.kI = HopperConstants::hopperI;
    hopperConfig.Slot0.kD = HopperConstants::hopperD;
    hopperConfig.Slot0.kG = HopperConstants::hopperG;

    hopperMotor.GetConfigurator().Apply(hopperConfig, kTimeoutMs);

};

void Hopper::rotateHopperDown() {
    
};

void Hopper::rotateHopperUp() {
    
};

void Hopper::stop() {

};