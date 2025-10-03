#include "htcw_rmt_ir.h"
#include <memory.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_encoder.h>

typedef struct {
	rmt_encoder_t base;
	rmt_encoder_t *copy_encoder;
	uint8_t bit_index;
	int state;
} rmt_ir_encoder_t;

typedef struct {
	rmt_ir_vendor_t irtype;
	uint32_t ircode;
	uint8_t bits;
} sendir_t;

typedef struct {
	uint16_t header_high;
	uint16_t header_low;
	uint16_t one_high;
	uint16_t one_low;
	uint16_t zero_high;
	uint16_t zero_low;
	uint16_t footer_high;
	uint8_t footer_low;
	uint16_t frequency;
	const char* name;
} ir_protocol_t;

typedef struct {
    rmt_channel_handle_t tx_handle;
    rmt_encoder_handle_t encoder_handle;
    rmt_ir_vendor_t brand;
} rmt_ir_send_dev_t;

typedef struct {
    rmt_channel_handle_t rx_handle;
    QueueHandle_t rx_queue;
    TaskHandle_t rx_task;
    rmt_ir_on_recv_callback_t callback;
    void* callback_state;
} rmt_ir_recv_dev_t;

static const ir_protocol_t proto[IR_MAX] = {
	[IR_UNK]  = {    0,    0,    0,    0,   0,   0,   0, 0,     0, "IR_UNK"  },
	[IR_NEC]  = { 9000, 4500,  560, 1690, 560, 560, 560, 0, 38000, "IR_NEC"  },
	[IR_SONY] = { 2400,  600, 1200,  600, 600, 600,   0, 0, 40000, "IR_SONY" },
	[IR_SAM]  = { 4500, 4500,  560, 1690, 560, 560, 560, 0, 38000, "IR_SAM"  },
	[IR_RC5]  = { 0,       0,  889,  889, 889, 889,   0, 0, 38000, "IR_RC5"  },
};


static const uint8_t  bitMargin = 120;
static volatile uint8_t irTX = 0;
static volatile uint8_t irRX = 0;


static void fill_item(rmt_symbol_word_t *item, uint16_t high, uint16_t low, bool bit){
	item->level0 = !bit;
	item->duration0 = high;
	item->level1 = bit;
	item->duration1 = low;
}

static size_t rmt_encode_ir(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
	rmt_ir_encoder_t *ir_encoder = __containerof(encoder, rmt_ir_encoder_t, base);
	
	rmt_encode_state_t fstate = RMT_ENCODING_RESET;
	
	size_t encoded_symbols = 0;
	rmt_encoder_handle_t copy_encoder = ir_encoder->copy_encoder;
	
	const sendir_t *send_data = (const sendir_t *)primary_data;
	ir_protocol_t protocol = proto[send_data->irtype];
	

	if (ir_encoder->state == 0) {
		if(protocol.header_high>0){
			rmt_symbol_word_t header_symbol;
			fill_item(&header_symbol, protocol.header_high, protocol.header_low, 0);
			encoded_symbols += copy_encoder->encode(copy_encoder, channel, &header_symbol, sizeof(rmt_symbol_word_t), &fstate);
		}else{
			fstate = RMT_ENCODING_COMPLETE;
		}
		
		if (fstate & RMT_ENCODING_COMPLETE) {
			ir_encoder->state = 1;
			ir_encoder->bit_index = 0;
		}
		if (fstate & RMT_ENCODING_MEM_FULL) {
			fstate = RMT_ENCODING_MEM_FULL;
			*ret_state = fstate;
			return encoded_symbols;
		}
	}

	if (ir_encoder->state == 1) {
	
		uint8_t rcspecial = 0;
		if(send_data->irtype == IR_RC5){rcspecial = 1;}
		
		rmt_symbol_word_t one_symbol;
		fill_item(&one_symbol, protocol.one_high, protocol.one_low, rcspecial);

		rmt_symbol_word_t zero_symbol;
		fill_item(&zero_symbol, protocol.zero_high, protocol.zero_low, 0);

		for (uint8_t i = ir_encoder->bit_index; i < send_data->bits; i++) {
			if (send_data->ircode & (1 << (send_data->bits - 1 - i))) {
				encoded_symbols += copy_encoder->encode(copy_encoder, channel, &one_symbol, sizeof(rmt_symbol_word_t), &fstate);
			} else {
				encoded_symbols += copy_encoder->encode(copy_encoder, channel, &zero_symbol, sizeof(rmt_symbol_word_t), &fstate);
			}
			if (fstate & RMT_ENCODING_MEM_FULL) {
				fstate = RMT_ENCODING_MEM_FULL;
				ir_encoder->bit_index = i + 1;
				*ret_state = fstate;
				return encoded_symbols;
			}
		}
		ir_encoder->state = 2;
	}

	if (ir_encoder->state == 2) {
		if(protocol.footer_high>0){
			rmt_symbol_word_t end_symbol;
			fill_item(&end_symbol, protocol.footer_high, protocol.footer_low, 0);
			encoded_symbols += copy_encoder->encode(copy_encoder, channel, &end_symbol, sizeof(rmt_symbol_word_t), &fstate);
		}else{
			fstate = (rmt_encode_state_t)((int)fstate | RMT_ENCODING_COMPLETE);
		}
		if (fstate & RMT_ENCODING_COMPLETE) {
			ir_encoder->state = 0;
			*ret_state = RMT_ENCODING_COMPLETE;
		}
	}

	return encoded_symbols;
}


esp_err_t rmt_del_ir_encoder(rmt_encoder_t *encoder) {
	rmt_ir_encoder_t *ir_encoder = __containerof(encoder, rmt_ir_encoder_t, base);
	rmt_del_encoder(ir_encoder->copy_encoder);
	free(ir_encoder);
	return ESP_OK;
}

esp_err_t rmt_ir_encoder_reset(rmt_encoder_t *encoder) {
	rmt_ir_encoder_t *ir_encoder = __containerof(encoder, rmt_ir_encoder_t, base);
	rmt_encoder_reset(ir_encoder->copy_encoder);
	ir_encoder->state = 0;
	ir_encoder->bit_index = 0;
	return ESP_OK;
}


bool rmt_ir_send_one(uint8_t pin, rmt_ir_vendor_t brand, uint32_t code, uint8_t bits, uint8_t burst, uint8_t repeat) {

	irTX = 1;
	for(;;){
		if(irRX == 0){break;}
		vTaskDelay( 2 / portTICK_PERIOD_MS );
	}

	sendir_t codetx = {brand, code, bits};

	rmt_channel_handle_t tx_channel = NULL;
	
	rmt_tx_channel_config_t txconf = {
		.gpio_num = (gpio_num_t)(pin),
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 1000000, // 1MHz resolution, 1 tick = 1us
		.mem_block_symbols = 64,
		.trans_queue_depth = 4,
	};
	
	if(rmt_new_tx_channel(&txconf, &tx_channel) != ESP_OK) {
		return false;
	}
	float duty = 0.50;
	uint8_t rptgap = 100; if(brand == IR_SONY){rptgap = 24; duty = 0.40;}
	
	rmt_carrier_config_t carrier_cfg = {
		.frequency_hz = proto[brand].frequency,
		.duty_cycle = duty,
		//.flags = {
		//	.polarity_active_low = 0
		//}
	};
	
	rmt_apply_carrier(tx_channel, &carrier_cfg);

	rmt_ir_encoder_t *ir_encoder = (rmt_ir_encoder_t *)malloc(sizeof(rmt_ir_encoder_t));
	memset(ir_encoder,0,sizeof(rmt_ir_encoder_t));
    ir_encoder->base.encode = rmt_encode_ir;
	ir_encoder->base.del = rmt_del_ir_encoder;
	ir_encoder->base.reset = rmt_ir_encoder_reset;
    
	rmt_copy_encoder_config_t copy_encoder_config = {};
	if(ESP_OK!=rmt_new_copy_encoder(&copy_encoder_config, &ir_encoder->copy_encoder)) {
        free(ir_encoder);
        return false;
    }

	rmt_encoder_handle_t encoder_handle = &ir_encoder->base;

	if(ESP_OK!=rmt_enable(tx_channel)) {
        free(ir_encoder);
        return false;
    }

	rmt_transmit_config_t tx_config = {
		.loop_count = 0,
		//.flags = {
		//	.eot_level = 0
		//}
	};
	for(uint8_t j = 0; j < repeat; j++){
		for(uint8_t i = 0; i < burst; i++){
			rmt_transmit(tx_channel, encoder_handle, &codetx, sizeof(codetx), &tx_config);
			rmt_tx_wait_all_done(tx_channel, portMAX_DELAY);
			vTaskDelay( rptgap / portTICK_PERIOD_MS );
		}
		vTaskDelay( 100 / portTICK_PERIOD_MS );
	}
	
	rmt_disable(tx_channel);
	rmt_del_channel(tx_channel);
	rmt_del_encoder(encoder_handle);
	irTX = 0;
    return true;
}

bool rmt_ir_send_create(uint8_t pin, rmt_ir_vendor_t brand, rmt_ir_send_handle_t* out_handle) {

	rmt_channel_handle_t tx_channel = NULL;
	
	rmt_tx_channel_config_t txconf = {
		.gpio_num = (gpio_num_t)(pin),
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 1000000, // 1MHz resolution, 1 tick = 1us
		.mem_block_symbols = 64,
		.trans_queue_depth = 4,
	};
	
	if(rmt_new_tx_channel(&txconf, &tx_channel) != ESP_OK) {
		return false;
	}
	float duty = 0.50;
	uint8_t rptgap = 100; if(brand == IR_SONY){rptgap = 24; duty = 0.40;}
	
	rmt_carrier_config_t carrier_cfg = {
		.frequency_hz = proto[brand].frequency,
		.duty_cycle = duty,
		//.flags = {
		//	.polarity_active_low = 0
		//}
	};
	
	rmt_apply_carrier(tx_channel, &carrier_cfg);

	rmt_ir_encoder_t *ir_encoder = (rmt_ir_encoder_t *)malloc(sizeof(rmt_ir_encoder_t));
	memset(ir_encoder,0,sizeof(rmt_ir_encoder_t));
    ir_encoder->base.encode = rmt_encode_ir;
	ir_encoder->base.del = rmt_del_ir_encoder;
	ir_encoder->base.reset = rmt_ir_encoder_reset;
    
	rmt_copy_encoder_config_t copy_encoder_config = {};
	if(ESP_OK!=rmt_new_copy_encoder(&copy_encoder_config, &ir_encoder->copy_encoder)) {
        free(ir_encoder);
        return false;
    }

	rmt_encoder_handle_t encoder_handle = &ir_encoder->base;
    rmt_ir_send_dev_t* dev = (rmt_ir_send_dev_t*)malloc(sizeof(rmt_ir_send_dev_t));
    dev->brand = brand;
    dev->encoder_handle = encoder_handle;
    dev->tx_handle = tx_channel;
    *out_handle = dev;
    return true;
}
void rmt_ir_send_del(rmt_ir_send_handle_t handle) {
    rmt_ir_send_dev_t* dev = (rmt_ir_send_dev_t*)handle;
    // rmt_disable(dev->tx_handle);
	rmt_del_channel(dev->tx_handle);
	rmt_del_encoder(dev->encoder_handle);
    free(dev);
}

bool rmt_ir_send(rmt_ir_send_handle_t handle, uint32_t code, uint8_t bits, uint8_t burst, uint8_t repeat) {
    rmt_ir_send_dev_t* dev = (rmt_ir_send_dev_t*)handle;
    if(ESP_OK!=rmt_enable(dev->tx_handle)) {
        return false;
    }
    sendir_t codetx = {dev->brand, code, bits};
    float duty = 0.50;
	uint8_t rptgap = 100; if(dev->brand == IR_SONY){rptgap = 24; duty = 0.40;}
	
	rmt_transmit_config_t tx_config = {
		.loop_count = 0,
	};
	for(uint8_t j = 0; j < repeat; j++){
		for(uint8_t i = 0; i < burst; i++){
			rmt_transmit(dev->tx_handle, dev->encoder_handle, &codetx, sizeof(codetx), &tx_config);
			rmt_tx_wait_all_done(dev->tx_handle, portMAX_DELAY);
			vTaskDelay( rptgap / portTICK_PERIOD_MS );
		}
		vTaskDelay( 100 / portTICK_PERIOD_MS );
	}
	
	rmt_disable(dev->tx_handle);   
}


static bool checkbit(const rmt_symbol_word_t *item, uint16_t high, uint16_t low){
	return item->level0 == 0 && item->level1 != 0 &&
		item->duration0 < (high + bitMargin) && item->duration0 > (high - bitMargin) &&
		item->duration1 < (low + bitMargin) && item->duration1 > (low - bitMargin);
}
static bool rc5_bit(uint32_t d, uint32_t v) {
	return (d < (v + bitMargin)) && (d > (v - bitMargin));
}

static uint32_t nec_check(const rmt_symbol_word_t *item, size_t* len){
	const uint8_t  totalData = 34;
	if(*len < totalData ){
		return 0;
	}
	const uint32_t m = 0x80000000;
	uint32_t code = 0;
	for(uint8_t i = 0; i < totalData; i++){
		if(i == 0){//header
			if(!checkbit(&item[i], proto[IR_NEC].header_high, proto[IR_NEC].header_low)){return 0;}
		}else if(i == 33){//footer
			if(!checkbit(&item[i], proto[IR_NEC].footer_high, proto[IR_NEC].footer_low)){return 0;}
		}else if(checkbit(&item[i], proto[IR_NEC].one_high, proto[IR_NEC].one_low)){
			code |= (m >> (i - 1) );
		}else if(!checkbit(&item[i], proto[IR_NEC].zero_high, proto[IR_NEC].zero_low)){
			return 0;
		}
	}
	return code;
}


static uint32_t sam_check(const rmt_symbol_word_t *item, size_t *len){
	const uint8_t  totalData = 34;
	if(*len < totalData ){
		return 0;
	}
	const uint32_t m = 0x80000000;
	uint32_t code = 0;
	for(uint8_t i = 0; i < totalData; i++){
		if(i == 0){//header
			if(!checkbit(&item[i], proto[IR_SAM].header_high, proto[IR_SAM].header_low)){return 0;}
		}else if(i == 33){//footer
			if(!checkbit(&item[i], proto[IR_SAM].footer_high, proto[IR_SAM].footer_low)){return 0;}
		}else if(checkbit(&item[i], proto[IR_SAM].one_high, proto[IR_SAM].one_low)){
			code |= (m >> (i - 1) );
		}else if(!checkbit(&item[i], proto[IR_SAM].zero_high, proto[IR_SAM].zero_low)){
			return 0;
		}
	}
	return code;
}

static uint32_t sony_check(const rmt_symbol_word_t *item, size_t* len){
	const uint8_t totalMin = 12;
	uint8_t i = 0;
	if(*len < totalMin || !checkbit(&item[i], proto[IR_SONY].header_high, proto[IR_SONY].header_low)){
		return 0;
	}
	i++;
	uint32_t m = 0x80000000;
	uint32_t code = 0;
	uint8_t maxData = 20;
	if(*len < maxData){maxData = 15;}
	if(*len < maxData){maxData = 12;}
	for(int j = 0; j < maxData - 1; j++){
		if(checkbit(&item[i], proto[IR_SONY].one_high, proto[IR_SONY].one_low)){
			code |= (m >> j);
		}else if(checkbit(&item[i], proto[IR_SONY].zero_high, proto[IR_SONY].zero_low)){
			code |= 0 & (m >> j);
		}else if(item[i].duration1 > 15000){ //space repeats
			break;
		}else{
			return 0;
		}
		i++;
	}
	code = code >> (32 - i);
	*len = i+1;
	return code;
}

static uint32_t rc5_check(const rmt_symbol_word_t *item, size_t *len){
	if(*len < 13 || *len > 30 ){
		return 0;
	}
	const uint16_t RC5_High = proto[IR_RC5].one_high + proto[IR_RC5].one_low;
	uint32_t code = 0; bool c = false;
	for(uint8_t i = 0; i < *len; i++){
		uint32_t d0 = item[i].duration0;
		uint32_t d1 = item[i].duration1;
		if (rc5_bit(d0, proto[IR_RC5].one_low)) {
			code = (code << 1) | c;
			c = rc5_bit(d1, RC5_High) ? !c : c;
		} else if (rc5_bit(d0, RC5_High)) {
			code = (code << 2) | (item[i].level0 << 1) | !item[i].level0;
			c = rc5_bit(d1, proto[IR_RC5].one_low) ? !c : c;
		}else{
			return 0;
		}
	}
	return code;
}

static bool irrx_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *udata){
	BaseType_t h = pdFALSE;
	QueueHandle_t q = (QueueHandle_t)udata;
	xQueueSendFromISR(q, edata, &h);
	return h == pdTRUE;
}

static void recv_task(void* param) {
    rmt_ir_recv_dev_t* dev = (rmt_ir_recv_dev_t*)param;
	rmt_rx_done_event_data_t rx_data;


    rmt_symbol_word_t symbols[64];
    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = 1250,
        .signal_range_max_ns = 12000000,
    };
    rmt_enable(dev->rx_handle);
    rmt_receive(dev->rx_handle, symbols, sizeof(symbols), &rx_config);
    for(;;){
        if (xQueueReceive(dev->rx_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS){
            size_t len = rx_data.num_symbols;
            rmt_symbol_word_t *rx_items = rx_data.received_symbols;

            if(len > 11){
                uint32_t rcode = 0; rmt_ir_vendor_t rproto = IR_UNK;
                if( rcode = nec_check(rx_items, &len) ){
                    rproto = IR_NEC;
                }else if( rcode = sony_check(rx_items, &len) ){
                    rproto = IR_SONY;
                }else if( rcode = sam_check(rx_items, &len) ){
                    rproto = IR_SAM;
                }else if( rcode = rc5_check(rx_items, &len) ){
                    rproto = IR_RC5;
                }
                if(dev->callback!=NULL) {
                    dev->callback(rproto,rcode,dev->callback_state);
                }
            } 
            rmt_receive(dev->rx_handle, symbols, sizeof(symbols), &rx_config);
        }
    }

}

bool rmt_ir_recv_create(uint8_t pin, rmt_ir_on_recv_callback_t callback, void* callback_state, rmt_ir_recv_handle_t* out_handle) {
    *out_handle = NULL;
    rmt_ir_recv_dev_t* dev = (rmt_ir_recv_dev_t*)malloc(sizeof(rmt_ir_recv_dev_t));
    if(dev==NULL) {
        return false;
    }
    rmt_rx_done_event_data_t rx_data; 
    QueueHandle_t rx_queue = xQueueCreate(1, sizeof(rx_data));

    rmt_channel_handle_t rx_channel = NULL;
    
    rmt_rx_channel_config_t rx_ch_conf = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 64,
    };
    
    if(ESP_OK!=rmt_new_rx_channel(&rx_ch_conf, &rx_channel)) {
        free(dev);
        vQueueDelete(rx_queue);
        return false;
    }
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = irrx_done,
    };

    if(ESP_OK!=rmt_rx_register_event_callbacks(rx_channel, &cbs, rx_queue)) {
        free(dev);
        vQueueDelete(rx_queue);
        rmt_del_channel(rx_channel);
        return false;
    }
    dev->rx_queue = rx_queue;
    dev->rx_handle = rx_channel;
    dev->callback = callback;
    dev->callback_state = callback_state;
    TaskHandle_t task = NULL;
    xTaskCreate(recv_task,"rmt_ir_recv_task",2048,dev,10,&task);
    if(task==NULL) {
        free(dev);
        vQueueDelete(rx_queue);
        rmt_del_channel(rx_channel);
        return false;
    }
    dev->rx_task = task;
    *out_handle = dev;
    return true;
}
void rmt_ir_recv_del(rmt_ir_recv_handle_t handle) {
    if(handle==NULL) {
        return;
    }
    rmt_ir_recv_dev_t* dev = (rmt_ir_recv_dev_t*)handle;
    dev->callback = NULL;
    vTaskDelete(dev->rx_task);
    rmt_disable(dev->rx_handle);
    vQueueDelete(dev->rx_queue);
    rmt_del_channel(dev->rx_handle);
    free(dev);

}