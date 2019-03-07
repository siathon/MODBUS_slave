#include "main.h"

int main() {
    rs.set_flow_control(Serial::Disabled);
    rs.attach(&rsRx);

    ev_queue.call_every(10, waitForPacket);
    ev_queue.call_every(500, blink);
    ev_queue.dispatch_forever();
}
