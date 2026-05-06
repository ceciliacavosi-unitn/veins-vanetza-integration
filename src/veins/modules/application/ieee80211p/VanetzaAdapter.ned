package veins.modules.application.ieee80211p;

//
// Modulo adattatore per l'integrazione della libreria Vanetza in Veins.
// Gestisce la logica di generazione CAM (ETSI TRC) e la serializzazione/decodifica
// dei messaggi V2X (CAM e DENM) verso il data provider del veicolo.
//
simple VanetzaAdapter
{
    parameters:
        @display("i=block/network2");
        @class(veins::VanetzaAdapter);
}