#ifndef _DW3000_CLI_PRIV_H_
#define _DW3000_CLI_PRIV_H_

#include <streamer/streamer.h>

void dw3000_cli_dump_registers(struct _dw3000_dev_instance_t * inst, struct streamer *streamer);
void dw3000_cli_dump_event_counters(struct _dw3000_dev_instance_t * inst, struct streamer *streamer);
void dw3000_cli_backtrace(struct _dw3000_dev_instance_t * inst, uint16_t verbose, struct streamer *streamer);
void dw3000_cli_spi_backtrace(struct _dw3000_dev_instance_t * inst, uint16_t verbose, struct streamer *streamer);
void dw3000_cli_interrupt_backtrace(struct _dw3000_dev_instance_t * inst, uint16_t verbose, struct streamer *streamer);
int dw3000_cli_cw(struct _dw3000_dev_instance_t *inst, int enable);
int dw3000_cli_autorx(struct _dw3000_dev_instance_t *inst, int enable);
int dw3000_cli_continuous_tx(struct _dw3000_dev_instance_t *inst, int frame_length, uint32_t delay_ns);


#if MYNEWT_VAL(DW3000_OTP_CLI)
void dw3000_cli_otp(struct _dw3000_dev_instance_t *inst, int argc, char **argv, struct streamer *streamer);
#endif

#endif /* _DW3000_CLI_PRIV_H_ */
