#ifndef __SPCD_CAMERA_SWITCH_H__
#define __SPCD_CAMERA_SWITCH_H__

_EXT_DTCM1_ROD const uint16_t XC7023_1080P_TO_720P[] = {
	0xfffe, 0x26,
	0x8010, 0x08,
	0xfffe, 0x50, //��ҳ
	0x0030, 0x05, //PLL
	0x0031, 0x06,
	0x0032, 0x13,

	0xfffe, 0x26,
	0x6006, 0xA,
	0x6007, 0x8C,
	0x6008, 0x9,

	0xfffe, 0x30,
	0x0001, 0xb1,
	0x000a, 0x5,
	0x000b, 0x0,
	0x000c, 0x2,
	0x000d, 0xD0,
	0x0027, 0xF1,
	0xfffe, 0x26,
	0x8010, 0x0d
};

_EXT_DTCM1_ROD const uint16_t XC7023_720P_TO_1080P[] = {
	0xfffe, 0x26,
	0x8010, 0x08,

	0xfffe, 0x50, //��ҳ
	0x0030, 0x09, //PLL
	0x0031, 0x02,
	0x0032, 0x0c,

	0xfffe, 0x26,
	0x6006, 0xF,
	0x6007, 0xA0,
	0x6008, 0xE,

	0xfffe, 0x30,
	0x0001, 0xa1,
	0x000a, 0x7,
	0x000b, 0x80,
	0x000c, 0x4,
	0x000d, 0x38,
	0x0027, 0xF7,

	0xfffe, 0x26,
	0x8010, 0x0d
};

#endif