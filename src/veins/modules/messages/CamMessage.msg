import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;

namespace veins;

packet CamMessage extends BaseFrame1609_4 {
    uint8_t data[];      // I byte reali generati da Vanetza
    int stationID;       // ID del veicolo per i log
    Coord senderPos;     // Posizione per il debug visivo
}