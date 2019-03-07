#include "main.h"

int main() {
    rs.set_flow_control(Serial::Disabled);
    ev_queue.call_every(500, blink);
    ev_queue.call_in(10000, sendPacket);
    ev_queue.call_every(300000, sendPacket);
    ev_queue.dispatch_forever();
}
