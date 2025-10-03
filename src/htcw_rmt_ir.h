#ifndef HTCW_RMT_IR_H
#define HTCW_RMT_IR_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { IR_UNK, IR_NEC, IR_SONY, IR_SAM, IR_RC5, IR_MAX } rmt_ir_vendor_t;
typedef void* rmt_ir_send_handle_t;
typedef void* rmt_ir_recv_handle_t;
typedef void(*rmt_ir_on_recv_callback_t)(rmt_ir_vendor_t brand, uint32_t code, void* state);
#ifdef __cplusplus
extern "C" {
#endif
bool rmt_ir_send_create(uint8_t pin, rmt_ir_vendor_t brand, rmt_ir_send_handle_t* out_handle);
void rmt_ir_send_del(rmt_ir_send_handle_t tx_handle);
bool rmt_ir_send(rmt_ir_send_handle_t handle, uint32_t code, uint8_t bits, uint8_t burst, uint8_t repeat);
bool rmt_ir_send_one(uint8_t pin, rmt_ir_vendor_t brand, uint32_t code, uint8_t bits, uint8_t burst, uint8_t repeat);

bool rmt_ir_recv_create(uint8_t pin, rmt_ir_on_recv_callback_t callback, void* callback_state, rmt_ir_recv_handle_t* out_handle);
void rmt_ir_recv_del(rmt_ir_recv_handle_t handle);
#ifdef __cplusplus
}
#endif
#endif // HTCW_RMT_IR_H