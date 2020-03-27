//#include "SandboxMassSpringDamper.h"
//#include "SandboxEngine.h"
//#include "SandboxTransmission.h"
//#include "SandboxVehicleStandalone.h"
//#include "SandboxFullModel.h"

#include "SimFramework/Utilities.h"



int main() {




    SimFramework::CSVWriter csv;

    csv.SetOutputFilepath("outCSV.csv");
    csv.AppendRow({"1", "2", "3"});
    csv.AppendRow({"2", "5", "3"});




//    SandboxMassSpringDamper();
//    SandboxEngine();
//    SandboxTransmission();
//    SandboxVehicleStandalone();
//    SandboxFullModel();


    return 0;
}
