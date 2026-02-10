#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/Turret.h"

namespace turret{
    frc2::CommandPtr setTurretAngle(Turret* m_turret);

}
