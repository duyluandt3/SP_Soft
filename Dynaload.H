#include	 <windows.h>
#include	 <winbase.h>

// 通信ﾓｰﾄﾞ設定値
#define		Xfer3M	0x00		// 3Mbps
#define		Xfer6M	0x01		// 6Mbps
#define		Xfer12M	0x02		// 12Mbps
#define		HalfDup	0x00		// Half Duplex
#define		FullDup	0x01		// Full Duplex
#define		EnbIrq	0x01		// IRQ出力enable
#define		DisIrq	0x00		// IRQ出力disable

// ﾀｰﾐﾅﾙ･ﾕﾆｯﾄｺｰﾄﾞ
#define		Unused	0x00		// 未接続
#define		In16	0x01		// 入力16点
#define		Out16	0x02		// 出力16点
#define		Io8		0x03		// I/O 8点 または I/O 16点
#define		Pulse	0x04		// ﾊﾟﾙｽ6点入力
#define		Ad8		0x05		// A/D ｱﾅﾛｸﾞ8点入力
#define		Da4		0x06		// D/A ｱﾅﾛｸﾞ4点出力
#define		MltAd	0x07		// Multi A/D ｱﾅﾛｸﾞ4点入力
#define		Da8		0x08		// D/A8ch ｱﾅﾛｸﾞ8点出力

// DA出力、AD入力データ変換係数
//#define	LSB		5/4095
#define		LSBX	(4095/5)

// Error Code & Error Code From Dll to API
#define SNMA_NO_ERROR			TRUE					// No Error
#define SNMA_INIT_ERROR			99						// ﾎﾞｰﾄﾞ初期化失敗
#define SNMA_INVALID_UNIT		ERROR_BAD_UNIT			// 無効なﾕﾆｯﾄ指定
#define SNMA_REQ_INVALID_DATA	ERROR_INVALID_DATA		// 要求ﾃﾞｰﾀが無効
#define SNMA_REQ_BAD_LENGTH		ERROR_BAD_LENGTH		// 要求ﾃﾞｰﾀ長が無効
#define SNMA_BAD_DEVICE			ERROR_BAD_DEVICE		// ﾎﾞｰﾄﾞ指定又はｱﾄﾞﾚｽ指定が無効

#pragma pack(push,1)

//	ﾎﾞｰﾄﾞ一覧の取得用構造体の宣言
typedef struct {
		BYTE BoardId;		// ﾎﾞｰﾄﾞ識別ID（ﾛｰﾀﾘｰDSW設定値)
		WORD CfgAddr;		// Local Configuration Register Base
		WORD BdAddr;		// SN-1001-PCIMA Base
		BYTE IrqNum;		// IRQ Level Setting
		BYTE XferRate;		// ﾃﾞｰﾀ転送速度（0-3Mbps,1-6Mbps,2-12Mbps)
		BYTE XferMode;		// ﾃﾞｰﾀ転送ﾓｰﾄﾞ(0-half duplex,1-full dupl-
							// ex)
		BYTE IrqEnb;		// H/W割込出力許可(0-禁止,1-許可)
	} CfgInfo;

typedef struct {
		WORD BrdListLen;	// これに続く設定情報の長さ
							// (認識しているﾎﾞｰﾄﾞ数)*(設定情報の長さ)
		CfgInfo Bd0Inf;		// ﾎﾞｰﾄﾞID0ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd1Inf;		// ﾎﾞｰﾄﾞID1ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd2Inf;		// ﾎﾞｰﾄﾞID2ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd3Inf;		// ﾎﾞｰﾄﾞID3ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd4Inf;		// ﾎﾞｰﾄﾞID4ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd5Inf;		// ﾎﾞｰﾄﾞID5ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd6Inf;		// ﾎﾞｰﾄﾞID6ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd7Inf;		// ﾎﾞｰﾄﾞID7ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd8Inf;		// ﾎﾞｰﾄﾞID8ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd9Inf;		// ﾎﾞｰﾄﾞID9ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd10Inf;	// ﾎﾞｰﾄﾞID10ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd11Inf;	// ﾎﾞｰﾄﾞID11ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd12Inf;	// ﾎﾞｰﾄﾞID12ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd13Inf;	// ﾎﾞｰﾄﾞID13ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd14Inf;	// ﾎﾞｰﾄﾞID14ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
		CfgInfo Bd15Inf;	// ﾎﾞｰﾄﾞID15ｺﾝﾌｨｸﾞﾚｰｼｮﾝ情報
	} BrdList;

//	ﾏﾙﾁA/Dﾃﾞｰﾀbit fieldの宣言
typedef union{
	struct{
		USHORT	VolCode		:12;
		USHORT	Reserved	:3;
		USHORT	flag		:1;
	}bfld;
	USHORT	stbin;
}MLTAD_CHDATA, *PMLTAD_CHDATA;

//	変換後、ﾏﾙﾁA/Dﾃﾞｰﾀ構造体の宣言
typedef struct{
		UCHAR	Error;		// ｴﾗｰﾌﾗｸﾞ
		UCHAR	Pol;		// 極性
		double	Analog;		// 電圧または電流絶対値
	}ANA_DATA, *PANA_DATA;

//	ﾏﾙﾁA/Dﾓｰﾄﾞ設定値
typedef	enum{
	V0_5, V1_5, V0_10, V_PN5, V_PN10, C0_20, C4_20, C_PN20
}MLTAD_MODE;

#pragma pack(pop)

#define	DllExport		__declspec( dllexport )

typedef DWORD	DllExport (WINAPI *OPENVXD)( void );					//VxDｵｰﾌﾟﾝ
typedef void	DllExport (WINAPI *CLOSEVXD)( void );					//VxDｸﾛｰｽﾞ
typedef DWORD	DllExport (WINAPI *BOARDINIT)( WORD );					//ﾎﾞｰﾄﾞ初期化
typedef DWORD	DllExport (WINAPI *TERMSTART)( WORD );					//通信開始
typedef DWORD 	DllExport (WINAPI *TERMSTOP)( WORD );					//通信停止
typedef DWORD 	DllExport (WINAPI *GETVERSION)(LPWORD);					//ﾄﾞﾗｲﾊﾞﾊﾞｰｼﾞｮﾝ取得
typedef DWORD 	DllExport (WINAPI *TERMCHK)( WORD, WORD, LPINT *);		//通信状態ﾁｪｯｸ
typedef DWORD	DllExport (WINAPI *BOARDCHK)( WORD, LPINT *);			//ﾎﾞｰﾄﾞ自身のﾁｪｯｸ
typedef	DWORD	DllExport (WINAPI *UNITCNTR)( WORD, WORD, WORD);		//ﾕﾆｯﾄｺﾝﾄﾛｰﾙ
typedef	DWORD	DllExport (WINAPI *DATAIN)( WORD, WORD, LPINT *);		//ﾃﾞｰﾀ入力
typedef	DWORD 	DllExport (WINAPI *DATAOUT)( WORD, WORD, WORD);			//ﾃﾞｰﾀ出力
typedef	DWORD	DllExport (WINAPI *DATAOUTW)( WORD, WORD, LPINT *);		//出力ﾃﾞｰﾀ読み返し
typedef	DWORD 	DllExport (WINAPI *ADIN)( WORD, WORD, LPINT *);			//A/Dﾃﾞｰﾀ入力
typedef	DWORD 	DllExport (WINAPI *DAOUT)( WORD, WORD, WORD, WORD);		//D/A出力
typedef	DWORD 	DllExport (WINAPI *CNTIN)( WORD, WORD, LPINT *);		//パルスｶｳﾝﾄ取得
typedef	DWORD 	DllExport (WINAPI *CNT0_)( WORD, WORD, WORD);			//ｶｳﾝﾄｸﾘｱ
typedef	DWORD	DllExport (WINAPI *TERMREAD)( WORD, LPINT *);			//ﾏｯﾌﾟ情報読み込み
typedef	DWORD	DllExport (WINAPI *TERMWRITE)( WORD, LPINT *);			//ﾏｯﾌﾟ情報書き込み
typedef	DWORD	DllExport (WINAPI *RETRYREAD)( WORD, LPINT *);			//ﾘﾄﾗｲ回数読み込み
typedef	DWORD	DllExport (WINAPI *RETRYWRITE)( WORD, LPINT *);			//ﾘﾄﾗｲ回数書き込み
typedef	DWORD 	DllExport (WINAPI *WIREREAD)( WORD, LPINT *);			//ﾜｲｱﾘﾝｸﾞ情報読み込み
typedef	DWORD 	DllExport (WINAPI *WIREWRITE)( WORD, LPINT *);			//ﾜｲｱﾘﾝｸﾞ情報書き込み
typedef	DWORD	DllExport (WINAPI *EEPROMWRITE)( WORD);					//EEPROM書き込み
typedef	DWORD	DllExport (WINAPI *WIRESWITCH)( WORD);					//WIREon関数と機能は同じ
typedef	DWORD 	DllExport (WINAPI *WIREON)( WORD);						//ﾜｲｱﾘﾝｸﾞｽﾀｰﾄ
typedef	DWORD 	DllExport (WINAPI *WIREOFF)( WORD);						//ﾜｲｱﾘﾝｸﾞｽﾄｯﾌﾟ
typedef	DWORD	DllExport (WINAPI *WIRECHK)( WORD, LPINT *);			//ﾜｲｱﾘﾝｸﾞ状態ﾁｪｯｸ
typedef	DWORD 	DllExport (WINAPI *BOARDLIST)(BrdList *, LPINT *);		//ﾎﾞｰﾄﾞの情報取得
typedef	DWORD 	DllExport (WINAPI *TERMSETUP)( WORD, BYTE, BYTE, BYTE);	//ﾎﾞｰﾄﾞの設定変更
typedef	DWORD	DllExport (WINAPI *MLTADIN)( WORD, WORD, PMLTAD_CHDATA);//ﾏﾙﾁA/Dﾃﾞｰﾀ入力
typedef	DWORD	DllExport (WINAPI *GETMLTADDATA)( WORD, MLTAD_MODE,
 								PMLTAD_CHDATA, PANA_DATA);				//ﾏﾙﾁA/Dﾃﾞｰﾀ変換

// ｸﾞﾛｰﾊﾞﾙ変数
extern	HINSTANCE	hDll;

// DLL関数ﾎﾟｲﾝﾀ
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

