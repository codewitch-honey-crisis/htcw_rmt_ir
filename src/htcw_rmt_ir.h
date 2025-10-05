#ifndef HTCW_RMT_IR_H
#define HTCW_RMT_IR_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
/// @brief The vendor of the signal
typedef enum { 
    /// @brief Unknown vendor
    IR_UNK, 
    /// @brief NEC
    IR_NEC, 
    /// @brief Sony
    IR_SONY, 
    /// @brief Samsung
    IR_SAM, 
    /// @brief RC5
    IR_RC5, 
    IR_MAX 
} rmt_ir_vendor_t;
/// @brief A send handle
typedef void* rmt_ir_send_handle_t;
/// @brief A receive handle
typedef void* rmt_ir_recv_handle_t;

/// @brief A callback to receive messages
typedef void(*rmt_ir_on_recv_callback_t)(rmt_ir_vendor_t brand, uint32_t code, size_t size_bits, void* state);
#ifdef __cplusplus
extern "C" {
#endif
/// @brief Creates a handle with which to send IR remote signals
/// @param pin The pin to use
/// @param brand The vendor
/// @param out_handle The returned handle
/// @return True on success, otherwise false
bool rmt_ir_send_create(uint8_t pin, rmt_ir_vendor_t brand, rmt_ir_send_handle_t* out_handle);
/// @brief Deletes a handle created with rmt_ir_send_create()
/// @param handle The handle
void rmt_ir_send_del(rmt_ir_send_handle_t handle);
/// @brief Sends an IR code
/// @param handle The handle created with rmt_ir_send_create()
/// @param code The code to send
/// @param size_bits The number of bits for the code
/// @param burst The burst count
/// @param repeat The repeat count
/// @return True on success, otherwise false
bool rmt_ir_send(rmt_ir_send_handle_t handle, uint32_t code, size_t size_bits, size_t burst, size_t repeat);
/// @brief Sends an IR signal without creating a handle
/// @param pin The pin to send on
/// @param brand The vendor
/// @param code The code
/// @param size_bits The length of the code in bits
/// @param burst The burst count
/// @param repeat The repeat count
/// @return True on success, otherwise false
/// @remarks It's more efficient if sending multiple codes to use a handle
bool rmt_ir_send_one(uint8_t pin, rmt_ir_vendor_t brand, uint32_t code, size_t size_bits, size_t burst, size_t repeat);
/// @brief Creates a receive channel for IR signals
/// @param pin The pin
/// @param callback The callback, or NULL if only polling
/// @param callback_state The user defined callback state, or NULL if unused
/// @param out_handle The created handle
/// @return True on success, otherwise false
bool rmt_ir_recv_create(uint8_t pin, rmt_ir_on_recv_callback_t callback, void* callback_state, rmt_ir_recv_handle_t* out_handle);
/// @brief Polls for an incoming code
/// @param handle The receive handle created with rmot_ir_recv_create()
/// @param out_brand The vendor id of the code
/// @param out_code The code
/// @param out_size_bits The length of the code in bits
/// @return True if there was a code, otherwise false
/// @remarks If you don't poll fast enough, you may drop messages. Use the callback to ensure you receive everything, but exclusively polling uses less resources
bool rmt_ir_recv_poll(rmt_ir_recv_handle_t handle, rmt_ir_vendor_t* out_brand, uint32_t* out_code, size_t* out_size_bits);
/// @brief Deletes a handle created with rmt_ir_recv_create()
/// @param handle The handle to delete
void rmt_ir_recv_del(rmt_ir_recv_handle_t handle);
#ifdef __cplusplus
}
#endif
#endif // HTCW_RMT_IR_H