#include "SandboxMassSpringDamper.h"
#include "SandboxLockupClutch.h"
#include "SandboxFullModel.h"

#include "ReportResults_Mass.h"
#include "ReportResults_Aero.h"
#include "ReportResults_RR.h"
#include "ReportResults_TransmissionAndClutch.h"


int main() {

    ReportResults_Mass();
    ReportResults_Aero();
    ReportResults_RR();
//    ReportResults_TransmissionAndClutch();



//    SandboxMassSpringDamper();
//    SandboxLockupClutch();
//    SandboxFullModel();




    return 0;
}
