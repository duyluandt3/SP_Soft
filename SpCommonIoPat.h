#ifndef	SP_COMMON_IO_PAT_H
#define	SP_COMMON_IO_PAT_H

typedef struct{
	WORD	wOutAdr;		//出力側のHLSアドレス
	WORD	wInAdr;			//入力側のHLSアドレス
	WORD	wTestBitNum;	//テストするビット数
	WORD	wOutBit[16];	//出力テストのビット順
	WORD	wInBit[16];		//出力ビットに対する入力側のビット順
}DIO_AUTO_TEST_TABLE;

enum{
	ADR_DO_48=48,
	ADR_DO_49,
	ADR_DO_50,
	ADR_DO_51,
	ADR_DO_52,
	ADR_DO_53,
	ADR_DO_54,
	ADR_DO_55,
	ADR_DO_56,
	ADR_DO_57,
	ADR_DO_58,
};

enum{
	ADR_DI_48=48,
	ADR_DI_49,
	ADR_DI_50,
	ADR_DI_51,
	ADR_DI_52,
	ADR_DI_53,
	ADR_DI_54,
	ADR_DI_55,
	ADR_DI_56,
	ADR_DI_57,
	ADR_DI_58,
};

enum{
	TEST_PATTERN0,
	TEST_PATTERN1,
	TEST_PATTERN2,
	TEST_PATTERN3,
	TEST_PATTERN4,
	TEST_PATTERN5,
	TEST_PATTERN6,
	TEST_PATTERN7,
	TEST_PATTERN8,
	TEST_PATTERN9,
	TEST_PATTERN10,
	TEST_PATTERN11,
	TEST_PATTERN12,
	TEST_PATTERN13,
	TEST_PATTERN14,
	TEST_PATTERN15,
	TEST_PATTERN16,
	TEST_PATTERN17,
	TEST_PATTERN18,
};
#define	ADR_DO_START			ADR_DO_48
#define	ADR_DO_END				ADR_DO_58
#define	SP_XXXX_SW1_START_ADR	(4)
#define	SP_XXXX_SW2_START_ADR	(8)
#define	SP_XXXX_SW3_START_ADR	(12)

#endif	SP_COMMON_IO_PAT_H
