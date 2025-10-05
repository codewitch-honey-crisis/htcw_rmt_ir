#include <Arduino.h>
#include <stdio.h>
#include "htcw_rmt_ir.h"
#ifdef RECV
void on_recv(rmt_ir_vendor_t brand, uint32_t code, size_t size_bits, void* state) {
    switch(brand) {
        case IR_SAM: fputs("Samsung",stdout); break;
        case IR_SONY: fputs("Sony",stdout); break;
        case IR_NEC: fputs("NEC",stdout); break;
        case IR_RC5: fputs("RC5",stdout); break;
        default: fputs("Unknown",stdout); break;
    }
    fprintf(stdout," 0x%8lx\n",code);
}
#endif
/*
  http://www.remotecentral.com/cgi-bin/mboard/rc-discrete/thread.cgi?5780
  https://github.com/z3t0/Arduino-IRremote/issues/263
  
  IR Codes
  
  Samsung TV
  Power SAMSUNG 0xE0E09966  On Only
  Power SAMSUNG 0xE0E040BF  Toggle
  CH+   SAMSUNG 0xE0E048B7
  CH-   SAMSUNG 0xE0E008F7
  V+    SAMSUNG 0xE0E0E01F
  V-    SAMSUNG 0xE0E0D02F
  Mute  SAMSUNG 0xE0E0F00F
  0xE0E0F00F = Mute
  0xE0E0E01F = Vol+
  0xE0E0D02F = Vol-
  0xE0E0BE41 = TV
  0xE0E0A35C = HDMI4 (Android)
  0xE0E09966 = Power On
  0xE0E09768 = HDMI1
  0xE0E09669 = PC
  0xE0E07D82 = HDMI2 (Satellite)
  0xE0E0619E = Component
  0xE0E043BC = HDMI3 (Freeview)
  0xE0E040BF = Power Toggle
  0xE0E037C8 = AV
  0xE0E021DE = EXT1
  0xE0E019E6 = Power Off
*/
void setup() {
    Serial.begin(115200);
#ifdef RECV
    rmt_ir_recv_handle_t recv_handle;
    if(!rmt_ir_recv_create(16,on_recv,NULL,&recv_handle)) {
        puts("Error creating recv handler");
    } else {
        puts("Created recv handler");
    }
#else
    rmt_ir_send_handle_t send_handle;
    if(!rmt_ir_send_create(16,IR_SAM,&send_handle)) {
        puts("Error creating send handle");
        return;
    } else {
        puts("Created send handle");
    }
    rmt_ir_send(send_handle,0xE0E09966,32,1,1);
    rmt_ir_send(send_handle,0xE0E09768,32,1,1);
    rmt_ir_send_del(send_handle);
#endif
}
void loop() {

}