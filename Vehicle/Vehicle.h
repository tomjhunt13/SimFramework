#ifndef FRAMEWORK_VEHICLE_H
#define FRAMEWORK_VEHICLE_H


namespace Vehicle {


    struct VehicleComponents;


    /*
     * Contains all blocks and signals required for full vehicle
     *
     *
     */

    class VehicleC {

    public:
        VehicleC();
        ~VehicleC();

        void Print();

        // TODO: Signal* GetSignal(std::string signalName);

        void Simulate(float t_np1) {};


    private:

        VehicleComponents* components;

    };

} // namespace Vehicle



#endif //FRAMEWORK_VEHICLE_H
