package veins.modules.application.ieee80211p;

// Bridge module between application layer and lower layers (NIC).
// Implemented in C++ as: veins::VanetzaAdapter
// The name must match Define_Module(VanetzaAdapter)

simple VanetzaAdapter
{
    gates:
        input upperIn;   // from application
        output upperOut; // to application

        input lowerIn;   // from NIC
        output lowerOut; // to NIC
}