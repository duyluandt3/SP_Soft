#include	 <windows.h>
#include	 <winbase.h>

// �ʐMӰ�ސݒ�l
#define		Xfer3M	0x00		// 3Mbps
#define		Xfer6M	0x01		// 6Mbps
#define		Xfer12M	0x02		// 12Mbps
#define		HalfDup	0x00		// Half Duplex
#define		FullDup	0x01		// Full Duplex
#define		EnbIrq	0x01		// IRQ�o��enable
#define		DisIrq	0x00		// IRQ�o��disable

// ����٥�Ưĺ���
#define		Unused	0x00		// ���ڑ�
#define		In16	0x01		// ����16�_
#define		Out16	0x02		// �o��16�_
#define		Io8		0x03		// I/O 8�_ �܂��� I/O 16�_
#define		Pulse	0x04		// ��ٽ6�_����
#define		Ad8		0x05		// A/D ��۸�8�_����
#define		Da4		0x06		// D/A ��۸�4�_�o��
#define		MltAd	0x07		// Multi A/D ��۸�4�_����
#define		Da8		0x08		// D/A8ch ��۸�8�_�o��

// DA�o�́AAD���̓f�[�^�ϊ��W��
//#define	LSB		5/4095
#define		LSBX	(4095/5)

// Error Code & Error Code From Dll to API
#define SNMA_NO_ERROR			TRUE					// No Error
#define SNMA_INIT_ERROR			99						// �ް�ޏ��������s
#define SNMA_INVALID_UNIT		ERROR_BAD_UNIT			// �������ƯĎw��
#define SNMA_REQ_INVALID_DATA	ERROR_INVALID_DATA		// �v���ް�������
#define SNMA_REQ_BAD_LENGTH		ERROR_BAD_LENGTH		// �v���ް���������
#define SNMA_BAD_DEVICE			ERROR_BAD_DEVICE		// �ް�ގw�薔�ͱ��ڽ�w�肪����

#pragma pack(push,1)

//	�ް�ވꗗ�̎擾�p�\���̂̐錾
typedef struct {
		BYTE BoardId;		// �ް�ގ���ID�i۰�ذDSW�ݒ�l)
		WORD CfgAddr;		// Local Configuration Register Base
		WORD BdAddr;		// SN-1001-PCIMA Base
		BYTE IrqNum;		// IRQ Level Setting
		BYTE XferRate;		// �ް��]�����x�i0-3Mbps,1-6Mbps,2-12Mbps)
		BYTE XferMode;		// �ް��]��Ӱ��(0-half duplex,1-full dupl-
							// ex)
		BYTE IrqEnb;		// H/W�����o�͋���(0-�֎~,1-����)
	} CfgInfo;

typedef struct {
		WORD BrdListLen;	// ����ɑ����ݒ���̒���
							// (�F�����Ă����ް�ސ�)*(�ݒ���̒���)
		CfgInfo Bd0Inf;		// �ް��ID0��̨��ڰ��ݏ��
		CfgInfo Bd1Inf;		// �ް��ID1��̨��ڰ��ݏ��
		CfgInfo Bd2Inf;		// �ް��ID2��̨��ڰ��ݏ��
		CfgInfo Bd3Inf;		// �ް��ID3��̨��ڰ��ݏ��
		CfgInfo Bd4Inf;		// �ް��ID4��̨��ڰ��ݏ��
		CfgInfo Bd5Inf;		// �ް��ID5��̨��ڰ��ݏ��
		CfgInfo Bd6Inf;		// �ް��ID6��̨��ڰ��ݏ��
		CfgInfo Bd7Inf;		// �ް��ID7��̨��ڰ��ݏ��
		CfgInfo Bd8Inf;		// �ް��ID8��̨��ڰ��ݏ��
		CfgInfo Bd9Inf;		// �ް��ID9��̨��ڰ��ݏ��
		CfgInfo Bd10Inf;	// �ް��ID10��̨��ڰ��ݏ��
		CfgInfo Bd11Inf;	// �ް��ID11��̨��ڰ��ݏ��
		CfgInfo Bd12Inf;	// �ް��ID12��̨��ڰ��ݏ��
		CfgInfo Bd13Inf;	// �ް��ID13��̨��ڰ��ݏ��
		CfgInfo Bd14Inf;	// �ް��ID14��̨��ڰ��ݏ��
		CfgInfo Bd15Inf;	// �ް��ID15��̨��ڰ��ݏ��
	} BrdList;

//	���A/D�ް�bit field�̐錾
typedef union{
	struct{
		USHORT	VolCode		:12;
		USHORT	Reserved	:3;
		USHORT	flag		:1;
	}bfld;
	USHORT	stbin;
}MLTAD_CHDATA, *PMLTAD_CHDATA;

//	�ϊ���A���A/D�ް��\���̂̐錾
typedef struct{
		UCHAR	Error;		// �װ�׸�
		UCHAR	Pol;		// �ɐ�
		double	Analog;		// �d���܂��͓d����Βl
	}ANA_DATA, *PANA_DATA;

//	���A/DӰ�ސݒ�l
typedef	enum{
	V0_5, V1_5, V0_10, V_PN5, V_PN10, C0_20, C4_20, C_PN20
}MLTAD_MODE;

#pragma pack(pop)

#define	DllExport		__declspec( dllexport )

typedef DWORD	DllExport (WINAPI *OPENVXD)( void );					//VxD�����
typedef void	DllExport (WINAPI *CLOSEVXD)( void );					//VxD�۰��
typedef DWORD	DllExport (WINAPI *BOARDINIT)( WORD );					//�ް�ޏ�����
typedef DWORD	DllExport (WINAPI *TERMSTART)( WORD );					//�ʐM�J�n
typedef DWORD 	DllExport (WINAPI *TERMSTOP)( WORD );					//�ʐM��~
typedef DWORD 	DllExport (WINAPI *GETVERSION)(LPWORD);					//��ײ���ް�ޮݎ擾
typedef DWORD 	DllExport (WINAPI *TERMCHK)( WORD, WORD, LPINT *);		//�ʐM�������
typedef DWORD	DllExport (WINAPI *BOARDCHK)( WORD, LPINT *);			//�ް�ގ��g������
typedef	DWORD	DllExport (WINAPI *UNITCNTR)( WORD, WORD, WORD);		//�Ưĺ��۰�
typedef	DWORD	DllExport (WINAPI *DATAIN)( WORD, WORD, LPINT *);		//�ް�����
typedef	DWORD 	DllExport (WINAPI *DATAOUT)( WORD, WORD, WORD);			//�ް��o��
typedef	DWORD	DllExport (WINAPI *DATAOUTW)( WORD, WORD, LPINT *);		//�o���ް��ǂݕԂ�
typedef	DWORD 	DllExport (WINAPI *ADIN)( WORD, WORD, LPINT *);			//A/D�ް�����
typedef	DWORD 	DllExport (WINAPI *DAOUT)( WORD, WORD, WORD, WORD);		//D/A�o��
typedef	DWORD 	DllExport (WINAPI *CNTIN)( WORD, WORD, LPINT *);		//�p���X���Ď擾
typedef	DWORD 	DllExport (WINAPI *CNT0_)( WORD, WORD, WORD);			//���ĸر
typedef	DWORD	DllExport (WINAPI *TERMREAD)( WORD, LPINT *);			//ϯ�ߏ��ǂݍ���
typedef	DWORD	DllExport (WINAPI *TERMWRITE)( WORD, LPINT *);			//ϯ�ߏ�񏑂�����
typedef	DWORD	DllExport (WINAPI *RETRYREAD)( WORD, LPINT *);			//��ײ�񐔓ǂݍ���
typedef	DWORD	DllExport (WINAPI *RETRYWRITE)( WORD, LPINT *);			//��ײ�񐔏�������
typedef	DWORD 	DllExport (WINAPI *WIREREAD)( WORD, LPINT *);			//ܲ��ݸޏ��ǂݍ���
typedef	DWORD 	DllExport (WINAPI *WIREWRITE)( WORD, LPINT *);			//ܲ��ݸޏ�񏑂�����
typedef	DWORD	DllExport (WINAPI *EEPROMWRITE)( WORD);					//EEPROM��������
typedef	DWORD	DllExport (WINAPI *WIRESWITCH)( WORD);					//WIREon�֐��Ƌ@�\�͓���
typedef	DWORD 	DllExport (WINAPI *WIREON)( WORD);						//ܲ��ݸ޽���
typedef	DWORD 	DllExport (WINAPI *WIREOFF)( WORD);						//ܲ��ݸ޽į��
typedef	DWORD	DllExport (WINAPI *WIRECHK)( WORD, LPINT *);			//ܲ��ݸޏ������
typedef	DWORD 	DllExport (WINAPI *BOARDLIST)(BrdList *, LPINT *);		//�ް�ނ̏��擾
typedef	DWORD 	DllExport (WINAPI *TERMSETUP)( WORD, BYTE, BYTE, BYTE);	//�ް�ނ̐ݒ�ύX
typedef	DWORD	DllExport (WINAPI *MLTADIN)( WORD, WORD, PMLTAD_CHDATA);//���A/D�ް�����
typedef	DWORD	DllExport (WINAPI *GETMLTADDATA)( WORD, MLTAD_MODE,
 								PMLTAD_CHDATA, PANA_DATA);				//���A/D�ް��ϊ�

// ��۰��ٕϐ�
extern	HINSTANCE	hDll;

// DLL�֐��߲��
extern	OPENVXD		OpenVxD;
extern	CLOSEVXD	CloseVxD;
extern	GETVERSION	GETversion;
extern	BOARDINIT	BOARDinit;
extern	TERMSTART	TERMstart;
extern	TERMSTOP	TERMstop;
extern	TERMCHK		TERMchk;
extern	BOARDCHK	BOARDchk;
extern	UNITCNTR	UNITcntr;
extern	DATAIN		DATAin;
extern	DATAOUT		DATAout;
extern	DATAOUTW	DATAoutW;
extern	ADIN		ADin;
extern	DAOUT		DAout;
extern	CNTIN		CNTin;
extern	CNT0_		CNT0;
extern	TERMREAD	TERMread;
extern	TERMWRITE	TERMwrite;
extern	RETRYREAD	RETRYread;
extern	RETRYWRITE	RETRYwrite;
extern	WIREREAD	WIREread;
extern	WIREWRITE	WIREwrite;
extern	EEPROMWRITE	EEPROMwrite;
extern	WIRESWITCH	WIREswitch;
extern	WIREON		WIREon;
extern	WIREOFF		WIREoff;
extern	WIRECHK		WIREchk;
extern	BOARDLIST	BOARDlist;
extern	TERMSETUP	TERMsetup;
extern	MLTADIN		MltAdIn;
extern	GETMLTADDATA GetMltAdData;
