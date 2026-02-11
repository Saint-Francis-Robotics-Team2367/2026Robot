/*
#include "commands/TurretCommand.h"
#include "subsystems/Turret.h"
#include <frc2/command/Commands.h>

frc2::CommandPtr turret::setTurretAngle(Turret* m_turret, double setpoint){
   // Register that this command requires the subsystem.
    return m_turret-> RunOnce(
        [m_turret, setpoint] {m_turret->setAngle(setpoint);} );        
};
*/