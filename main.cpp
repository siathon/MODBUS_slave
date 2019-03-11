#include "main.h"

int main() {
    rs.attach(&rsRx);
    wd.Configured(3);

    ev_queue.call_every(10, waitForPacket);
    ev_queue.call_every(500, blink);
    ev_queue.dispatch_forever();
}
