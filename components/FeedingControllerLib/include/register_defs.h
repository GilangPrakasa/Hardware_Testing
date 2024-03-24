#ifndef REGISTER_DEFS_H
#define REGISTER_DEFS_H

#define VMAJ_ADDR 0x00      // * Version major register
#define VMIN_ADDR 0x01      // * Version minor register
#define VPAT_ADDR 0x02      // * Version patch register

#define DMCV_ADDR 0x04
#define DOCT_ADDR 0x05
#define TMCV_ADDR 0x06
#define TOCT_ADDR 0x07

#define TCNR_ADDR 0x08      // * Thrower config register
#define DCNR_ADDR 0x0A      // * Dosing config register
#define TOCR_ADDR 0x09      // * Thrower control register
#define DOCR_ADDR 0x0B      // * Dosing control register
#define ASTR_ADDR 0x0C      // * Alert status register
#define AENR_ADDR 0x0D      // * Alert enable register
#define AOSR_ADDR 0x0E      // * Alert output select register

#define VERS_BITS 0x00      // * Feeding controller version bits

#define OENA_BITS 0x01      // * Driver output enable bits
#define PDCS_BITS 0x1F      // * PWM dutycycle config bits
#define OFSL_BITS 0xE0      // * Output frequency select bits

#define TOVC_BITS 0x01      // * Thrower overcurrent bits
#define TNLC_BITS 0x02      // * Thrower noLoadcurrent bits
#define TNDT_BITS 0x04      // * Thrower notDetected bits
#define DOVC_BITS 0x08      // * Dosing overcurrent bits
#define DNLC_BITS 0x10      // * Dosing noLoadcurrent bits
#define DNDT_BITS 0x20      // * Dosing notDetected bits
#define POVR_BITS 0x40      // * Power overvoltage bits
#define PUND_BITS 0x80      // * Power undervoltage bits

#define VERS_MASK 0xFF      // * Feeding controller version mask

#define OENA_MASK 0xFE      // * Driver output enable mask
#define PDCS_MASK 0xE0      // * PWM dutycycle config mask
#define OFSL_MASK 0x1F      // * Output frequency select mask

#define TOVC_MASK 0xFE      // * Thrower overcurrent mask
#define TNLC_MASK 0xFD      // * Thrower noLoadcurrent mask
#define TNDT_MASK 0xFB      // * Thrower notDetected mask
#define DOVC_MASK 0xF7      // * Dosing overcurrent mask
#define DNLC_MASK 0xEF      // * Dosing noLoadcurrent mask
#define DNDT_MASK 0xDF      // * Dosing notDetected mask
#define POVR_MASK 0xBF      // * Power overvoltage mask
#define PUND_MASK 0x7F      // * Power undervoltage mask

#define OCTV_MASK 0x00
#define OCDV_MASK 0x00

#define DMCV_MASK 0x00
#define TMCV_MASK 0x00

#endif