//////////////////////////////////////////////////////////////////////////
//
//	SP_XXXX_BOARD ピン治具対応　検査プログラム
//
// --------対応予定機種--------
//  SPIN_MOTHER_BOARD
//  SP_UPPER_IF_BOARD
//  SP_UPPER_IF_BOARD2
//  SP_FRONT_IF_BOARD
//  SP_FRONT_IF_BOARD2
//  SP_MOTHER_BOARD2
//
//////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <conio.h>
#include <TCHAR.h>
#include <time.h>

#include "DynaLoad.h"
#include "RS232_Source/HP34401A.h"

#include <shlwapi.h>
#pragma comment(lib, "shlwapi.lib")

//CONTEC AIO-160802AY-USBライブラリを使用する
#include "Caio.h"
#pragma comment(lib, "CAIO.LIB")

#include "HlsTestPat.h"
#include "SpCommonIoPat.h"
#include "SpinMotherIoPat.h"
#include "SpFront2IoPat.h"
#include "SpUpper1IoPat.h"
#include "SpUpper2IoPat.h"

//#define	_RS_DEBUG_MODE

#define	SOFTWARE_VERSION				"1.00"
#define HLS_COMM						(1)

#define	AD_OK_RATE						(0.25)
#define	DA_OK_RATE						(0.25)

#define LAST_TERM_ADR					60

#define	HARDWARE_FACTOR_20MA_GAIN		(0x5400)	//ハード上の理論的な係数（SP_FRONT_IF=0x5400）
#define	HARDWARE_FACTOR_4MA_OFFSET		(0x35C7)	//ハード上の理論的な4mA入力値（SP_FRONT_IF=0x35C7）

#define	HARDWARE_FACTOR_10V_GAIN		(0x4472)	//ハード上の理論的な係数（SP_MOTHER）
#define	HARDWARE_FACTOR_5V_GAIN			(0x4333)	//ハード上の理論的な係数（SP_UPPER）
#define	HARDWARE_FACTOR_0V_OFFSET		(0x505)		//ハード上の理論的な0V入力値（SP_UPPER_IF, SP_MOTHER）

#define	INVALID_ADRAW_MINVAL_4mA		(0x3000-0x1000)
#define	INVALID_ADRAW_MAXVAL_4mA		(0x3000+0x1000)
#define	INVALID_ADRAW_MINVAL_20mA		(0xF860-0x1000)
#define	INVALID_ADRAW_MAXVAL_20mA		(0xFFFF)	//(0xF860+0x1000)

#define	INVALID_ADRAW_MINVAL_F0V		(0x0500-0x0500)		//0-5V(0V入力)生値の警告閾値は調整すること
#define	INVALID_ADRAW_MAXVAL_F0V		(0x0500+0x1000)		//0-5V(0V入力)生値の警告閾値は調整すること
#define	INVALID_ADRAW_MINVAL_F5V		(0xF430-0x1000)		//0-5V(5V入力)生値の警告閾値は調整すること
#define	INVALID_ADRAW_MAXVAL_F5V		(0xFFFF)	//(0xF430+0x0500)		//0-5V(5V入力)生値の警告閾値は調整すること

#define	INVALID_ADRAW_MINVAL_T0V		(0x0500-0x0500)		//0-10V(0V入力)生値の警告閾値は調整すること
#define	INVALID_ADRAW_MAXVAL_T0V		(0x0500+0x1000)		//0-10V(0V入力)生値の警告閾値は調整すること
#define	INVALID_ADRAW_MINVAL_T10V		(0xF430-0x1000)		//0-10V(10V入力)生値の警告閾値は調整すること
#define	INVALID_ADRAW_MAXVAL_T10V		(0xFFFF)	//(0xF430+0x0500)		//0-10V(10V入力)生値の警告閾値は調整すること

#define	INVALID_DA_MINVAL_4mA			(4.0-2.0)
#define	INVALID_DA_MAXVAL_4mA			(4.0+2.0)
#define	INVALID_DA_MINVAL_20mA			(20.0-2.0)
#define	INVALID_DA_MAXVAL_20mA			(20.0+2.0)

#define	AD_SAMPL_NUM		128
#define	AD_INPUT_DATA_NUM	(AD_SAMPL_NUM+32)

#define	DA_SAMPL_NUM		16	//4
#define	DA_INPUT_DATA_NUM	(DA_SAMPL_NUM+0)
/*
typedef union{
	struct{
		unsigned short	B00	:1;
		unsigned short	B01	:1;
		unsigned short	B02	:1;
		unsigned short	B03	:1;
		unsigned short	B04	:1;
		unsigned short	B05	:1;
		unsigned short	B06	:1;
		unsigned short	B07	:1;
		unsigned short	B08	:1;
		unsigned short	B09	:1;
		unsigned short	B10	:1;
		unsigned short	B11	:1;
		unsigned short	B12	:1;
		unsigned short	B13	:1;
		unsigned short	B14	:1;
		unsigned short	B15	:1;
	}BIT;
	unsigned short	WORD;
}RELAY_WORD_BIT;
*/
#define	SN_ADR_RELAY_OUT_UNIT		(55)			//リレー制御用ターミナルアドレス（仮）
typedef union{
	struct{
		unsigned short	B00B03_ANALOG_CH	:4;		//アナログ+基板電源入出力ch (0x0:ch0～0xF:ch15)
		unsigned short	B04_ANALOG_MODE_IN	:1;		//出力(MultiMeter)測定モード (0:電流測定,1:電圧測定)
		unsigned short	B05_ANALOG_MODE_OUT	:1;		//D/A出力検査時の入力測定モード (0:電流測定,1:電圧測定)※SP基板では0:電流出力は無い
		unsigned short	B06_ANALOG_RES_VAL	:1;		//D/A出力負荷抵抗切り替え(0:負荷なし,1:電流測定時250Ω、電圧測定時1KΩ)※SP基板では0:電流出力は無い
		unsigned short	B07_ANALOG_IO_SL	:1;		//A/D入力、D/A出力検査切り替え(0:D/A出力、1:A/D入力)
		unsigned short	B08_PCB_PWR			:1;		//検査基板ON/OFF切り替え(0:OFF、1:ON)
		unsigned short	B09_CPLD_SEL		:1;		//CPLDの書き込みコネクタ選択(0:CN101=HLS側,1:CN99=USR側)
		unsigned short	B10_TST1			:1;		//TST1端子ON/OFF切り替え(0:OFF、1:ON)
		unsigned short	B11_TST2			:1;		//TST2端子ON/OFF切り替え(0:OFF、1:ON)
		unsigned short	B12_TST3			:1;		//TST3端子ON/OFF切り替え(0:OFF、1:ON)
		unsigned short	B13_TST4			:1;		//TST4端子ON/OFF切り替え(0:OFF、1:ON)
		unsigned short	B14_E8A_CONNECT		:1;		//E8Aデバッガ接続(0:切断、1:接続)
		unsigned short	B15_HLS_CONNECT		:1;		//HLS通信接続(0:切断、1:接続)
	}BIT;
	unsigned short	WORD;
}SN_CTL_WORD_BIT;
SN_CTL_WORD_BIT	m_snTestCtrlBit;

//アナログ入出力ch切り替え(リレー制御)
#define	ANA_INOUT_SEL_IN		(0)		//入力検査
#define	ANA_INOUT_SEL_OUT		(1)		//出力検査
#define	ANA_VI_SEL_VOLT			(0)		//電圧入出力検査
#define	ANA_VI_SEL_CURR			(1)		//電流入出力検査
#define	ANA_LOAD_SEL_OPEN		(0)		//抵抗負荷なし（OPEN）
#define	ANA_LOAD_SEL_RENBL		(1)		//抵抗負荷あり
void SnMultiMeterMeasureSel(WORD wANAch, int nInOutSel, int nVISel, int nLoadSel);

//基板電源・CPLD選択・E8A接続/切断・HLS通信ライン接続/断線切り替え(リレー制御)
void SnRelayOutputBit(int nBitSelect, WORD wData);		//1bitのみ切り替え可能版
void SnRelayOutputAllBit(WORD wPcbPwr, WORD wCpldSel, WORD wTst4En, WORD wE8aCon, WORD wHlsCon);	//全bit同時切り替え可能版

//基板の電源再投入
void PcbPwrReBoot(BOOL bRsMode);

void LogFolderCreate();
void LogPrintf(const char * Format, ...);
void LogPrintfCsv(const char *fname, const char * Format, ...);
//void SPxxxxAdjMain();
void KeyStrInputWait(char *strbuf);
int KeyInputWait();
int	KeyInputWaitYorN();
void KeyBufClear();
void HitAnyKeyWait();
void CreateLotFolder();
void GetAdMinMaxAve(DWORD *ValArray, DWORD *MinVal, DWORD *MaxVal, DWORD *AveVal);
void GetAdOffsetGain(int nAdRange, DWORD AdAveVal4mA, DWORD AdAveVal20mA, DWORD *dwAdOffset, DWORD *dwAdGain);
BOOL GetVoltageVal(HANDLE hhandle, double *val);
void GetDaMinMaxAve(double *ValArray, double *MinVal, double *MaxVal, double *AveVal);
#define DIO_TEST_ALL_BIT	(0xffff)
BOOL DioAutoTestSelPattern( DIO_AUTO_TEST_TABLE *iotbl, WORD wPatternNo, DWORD dwChkTime, BOOL bZeroChk, WORD wInMaskBit );
BOOL OtherUnitInputZeroChk( WORD wStartAdr, WORD wEndAdr, WORD wSkipAdr );
BOOL HlsOutStopTest( DIO_AUTO_TEST_TABLE *iotbl, WORD *wPatternNo, int nPatternCnt, DWORD dwChkTime );
BOOL DioLoopBackTest( WORD wOutAdr, WORD wOutBit, WORD wInAdr, WORD wInBit );

void SPxxxxMain();
void SPxxxxAllTestStart();
void SPxxxxTestSelectMenu();
BOOL SPxxxxSequenceTest(int nInsStartID, int nInsEndID);
void SnOutputInit(BOOL bAllUnit);
void SPxxxxGetBoardName();

#if (HLS_COMM==1)
BOOLEAN	LoadSnDll(const char *);		// ﾄﾞﾗｲﾊﾞDLLのﾛｰﾄﾞ及びｴﾝﾄﾘ取得
void ErrorExit(DWORD ErrorCode);		// ｴﾗｰ終了時の処理
void WaitProgressDot(DWORD Interval, DWORD DotMax);
void SnDout16(int nOutMode, WORD wOutval, DWORD dwWait);

// グローバル変数
BrdList     ListBuf;                // BOARDlist()で使用する情報読込みバッファ
WORD        BoardId;			    // Board ID
DWORD       termmap[64];		    // ターミナルマッピング情報の宣言
#endif

HINSTANCE   hDll = NULL;            // DLLのインスタンス・ハンドル
OPENVXD     OpenVxD;
CLOSEVXD    CloseVxD;
GETVERSION  GETversion;
BOARDINIT   BOARDinit;
TERMSTART   TERMstart;
TERMSTOP    TERMstop;
TERMCHK     TERMchk;
BOARDCHK    BOARDchk;
UNITCNTR    UNITcntr;
DATAIN      DATAin;
DATAOUT     DATAout;
DATAOUTW    DATAoutW;
ADIN        ADin;
DAOUT       DAout;
CNTIN       CNTin;
CNT0_       CNT0;
TERMREAD    TERMread;
TERMWRITE   TERMwrite;
RETRYREAD   RETRYread;
RETRYWRITE  RETRYwrite;
WIREREAD    WIREread;
WIREWRITE   WIREwrite;
EEPROMWRITE EEPROMwrite;
WIRESWITCH  WIREswitch;
WIREON      WIREon;
WIREOFF     WIREoff;
WIRECHK     WIREchk;
BOARDLIST   BOARDlist;
TERMSETUP   TERMsetup;
MLTADIN		MltAdIn;
GETMLTADDATA GetMltAdData;

#define	COM_STX						(0x02)
#define	COM_ETX						(0x03)
#define	SEND_RECV_PAKET_SIZE		(256)

//ボードID取得・設定
#define	SP_BID_CHECK_ONLY			(0x000)
#define	SP_BID_UPPER_IF_BOARD		(0x155)
#define	SP_BID_FRONT_IF_BOARD		(0x146)
#define	SP_BID_MOTHER_BOARD			(0x14D)

#define SP_RS_MODE_NORMAL			('0')
#define SP_RS_MODE_ADADJ			('1')
#define SP_RS_MODE_DAADJ			('3')
#define SP_RS_MODE_RAW				('4')
#define SP_RS_MODE_EEPSAVE			('F')
#define SP_FIXED_SEND_SIZE			(39)	//送信パケットサイズ（固定長）
//#define SP_FIXED_RECV_SIZE			(40)	//受信パケットサイズ（固定長）
#define SP_FIXED_RECV_SIZE			(48)	//受信パケットサイズ（固定長）
#define SP_FIXED_RECV_SIZE_OLD		(40)	//初期試作機Verのパケットサイズ（固定長）
#define SP_XXXX_ADCH_MAX			(6)
#define SP_XXXX_DACH_MAX			(3)

#define INSMODE_NONE				(-1)	//検査モード入力待ち
#define INSMODE_MANUAL				(0)		//手動検査
#define INSMODE_ALL					(1)		//全検査
#define INSMODE_AD_DA_ADJ			(2)		//AD,DA調整のみ
#define INSMODE_AD_DA_TEST			(3)		//AD,DA検査のみ

char		m_cLogFolderName[_MAX_PATH];	//ログ出力先フルパスファイル名
BOOLEAN     VxDOpen = FALSE;        // OpenVxDフラグ(TRUE -> open済み)
char		m_cComPort[128];				//
WORD        LastTermNum;			// 最終ターミナルアドレス
WORD		m_wSnDoutData[4];
char		pcBoardName[32];		// マスターボード名
int			m_nInspectMode = INSMODE_NONE;
char		m_strLogFileID[512];	//ログファイルの頭文字
char		m_strLotName[512];		//ロット名称（ログ保存先のフォルダ名称とする）
int			ForeverLoop=1;
int			m_nAutoSequence=-1;		//全テストシーケンス制御フラグ
DWORD		m_dwTestStartTick=0;		//検査開始時のTICKカウンタ
int			m_nIniPerformanceCounterUsed=0;

int			m_nInspectLoopTestCount = 0;
BOOL		m_bInspectLoopTest = FALSE;

HANDLE	m_hSpComHandle=INVALID_HANDLE_VALUE;
int		m_nSPxxxxBoardID=-1;
int		m_nIniBoardID=-1;
int		m_nIniSpFrontBoardType=0;
int		m_nIniSpUpperBoardType=0;
int		m_nIniSpMotherBoardType=0;
//int		m_nIniSpHlsTestDswAllOff=1;

int		m_nIniAO_0_0V = 0x8000;
int		m_nIniAO_2_5V;
int		m_nIniAO_5_0V;
int		m_nIniAO_10_0V;
int		m_nIniAO_0_OVR_L;
int		m_nIniAO_5_OVR_H;
int		m_nIniAO_10_OVR_H;
int		m_nIniAI_0_0V;
int		m_nIniAI_10_0V;
double	m_dContecAiAdjOffsetV;
double	m_dContecAiAdjGainV;

int		m_nIniAO_4mA;
int		m_nIniAO_12mA;
int		m_nIniAO_20mA;
int		m_nIniAO_4mA_OVR_L;
int		m_nIniAO_20mA_OVR_H;
int		m_nIniAI_4mA;
int		m_nIniAI_20mA;
double	m_dContecAiAdjOffsetI;
double	m_dContecAiAdjGainI;

char	m_cSPxxxxBoardName[32];
char	m_cSpxxComPort[128];				//
unsigned char	m_cSendPacket[SEND_RECV_PAKET_SIZE];
unsigned char	m_cRecvPacket[SEND_RECV_PAKET_SIZE];
int		m_nFirmVer=0;
char	m_strLogFileName[128];

BOOL	AioUnitOpen();
void	AioUnitClose();
void	AioUnitWriteAOVal(short sAOch, long lAOval);
void	AioUnitReadAIVal(short sAIch, double *dAIval);
char	m_cIniAioUnitDeviceName[32];
short	m_shAioUnitId = -1;
char	m_cAioStringWork[256];

BOOL RS_SpComOpen();
int RS_SpSendRecv(unsigned char *cSendPacket, int nSendSize, unsigned char *cRecvPacket, int nRecvSizeMax);
int RS_SpUpdateBoardID(int nNewBoardID, int *npFirmVer);
int RS_SpInputAdRawVal(int nch);
int RS_SpInputAdCorrectVal(int nch);
BOOL RS_SpSetAdOffsetGain(DWORD *dwAdOffset, DWORD *dwAdGain, DWORD *dwReadOffset, DWORD *dwReadGain);
BOOL RS_SpSetDaOffsetGain(DWORD *dwDaOffset, DWORD *dwDaGain, DWORD *dwReadOffset, DWORD *dwReadGain);
BOOL RS_SpUpdateEeprom(int nChMax);
BOOL RS_SpNormalMode();
unsigned char RS_SpCheckSum(unsigned char *buff, int nsize);
BOOL SpComChk(int nStartAdr, int nEndAdr, BOOL bCommChk);
BOOL SpMotherSxSwTest(int nSwNo, int nSwStart, WORD wInAdr, WORD wInStartBit, int nBitCount);
BOOL SnDiReadWait(long *plnDiWaitVal, DWORD dwFirstWaitMsec);	//入力データが安定して読み出せることを確認する

BOOL SPxxxxDC_PwrChk(int nInsID);
BOOL SPxxxxFirmWrite(int nInsID);
BOOL SPxxxxCPLDWrite(int nInsID, int nCpldId);
BOOL SPxxxxBIDSetting(int nInsID);
BOOL SPxxxxComChk(int nInsID, int nComID);
BOOL SPxxxxDioChkAuto(int nInsID, BOOL bManualTest, BOOL bAutoTest);
BOOL SPxxxxDioChkAutoSpinMother();
BOOL SPxxxxDioChkOtherSpinMother(int nInsID);
BOOL SPxxxxAdinChk(int nInsID, int nSch, int nEch);
BOOL SPxxxxAdInAdj(int nInsID, int nSch, int nEch);
BOOL SPxxxxLedTest(int nInsID);

BOOL SPxxxxDaOutAdj();
BOOL SPxxxxDaoutChk();
//BOOL SPxxxxComChk();
//BOOL SPxxxxAdinChk(int nSch, int nEch);
//BOOL SPxxxxAdInAdj(int nSch, int nEch);
BOOL SPxxxxDiChk();
BOOL SPxxxxDioChkManualUpper1();
BOOL SPxxxxDioChkManualUpper2();
BOOL SPxxxxDioChkAutoUpper1();
BOOL SPxxxxDioChkAutoUpper2();
BOOL SPxxxxDioChkAutoFront2();
//BOOL SPxxxxBIDSetting();

//SAVENETのbitを検査治具のMILコネクタピン番号に変換（メッセージ表示用）
int GetPinNumMilTerm(WORD wBitNum);
char* GetPinNumMilTerm2(WORD wAdr, WORD wBitNum);

#define	SP_SIGNAL_OUT		(0)
#define	SP_SIGNAL_IN		(1)
char* GetSpInternalSignalName(WORD wAdr, WORD wBitNum, char* strInOrOut);

char m_cMilConnectorPinName[16];
char m_cSpInternalSignalName[16];


LARGE_INTEGER	m_laFreq;
LARGE_INTEGER	m_laMsecCnt;
void	Sleep_Cnt( DWORD msec );		//パフォーマンスカウンタを利用したSleep関数

////////////////////////////////////////////////////////////////////
//
//          MAIN
//
////////////////////////////////////////////////////////////////////
void main( void )
{
#if (HLS_COMM==1)
	DWORD       err;
	WORD        version;            // 下位ドライバ（SYS/VxDバージョン番号)
	WORD        lpcnt;				// ループカウンタ
	DWORD       NumBoards;          // このプログラムが実行されるPC上に存在するマスターボードの枚数
	CfgInfo     *ListPtr;           // ボードコンフィグレーション情報ポインタ
	DWORD       retrycnt;           // リトライ回数設定値
#endif
	long	lBoardType, lCont;
	char	pcFileName[32];		// ファイル名 拡張子部分を除く

	// Frequency取得
	QueryPerformanceFrequency(&m_laFreq);
	m_laMsecCnt.QuadPart = m_laFreq.QuadPart / 1000;

    /////////////////////////////////////////////////
    // 変数初期化
    VxDOpen = FALSE;                // OpenVxD成功フラグ初期化
    LastTermNum = LAST_TERM_ADR;    // 最終ターミナルアドレス初期化
	pcFileName[0] = '\0';
	memset(m_wSnDoutData, 0, sizeof(m_wSnDoutData));
	
    LogPrintf("**** SP_XXXX_BOARD ピン治具対応 検査プログラム ****\n");
    LogPrintf("            " __DATE__ "  " __TIME__ " Version\n");

	LogFolderCreate();

    /////////////////////////////////////////////////
	// ボードタイプ選択
    /////////////////////////////////////////////////
//	printf("\n"
//		"Select Board Type\n"
//		"  1 : SN-1001-PCIMA\n"
//		"  2 : SN-1002-PCIMA\n"
//		"  3 : SN-1001-CAMA\n"
//		">>>>>>>>>> ");
	lCont = 1;
	while(lCont)
	{
//		lBoardType = _getche() - '0';
		lBoardType = 2;		//SN-1002-PCIMA固定
		switch(lBoardType)
		{
		case 1:
			strcpy(pcBoardName, "SN-1001-PCIMA");
			strcpy(pcFileName, "SN11PCI");
			lCont = 0;
			break;
		case 2:
			strcpy(pcBoardName, "SN-1002-PCIMA");
			strcpy(pcFileName, "SN12PCI");
			lCont = 0;
			break;
		case 3:
			strcpy(pcBoardName, "SN-1001-CAMA");
			strcpy(pcFileName, "SN11CAMA");
			lCont = 0;
			break;
		default:
			return;	// アプリ終了
		}
	}

	printf("\n");

    /////////////////////////////////////////////////
	//	検査ターミナル最終IDを入力
    /////////////////////////////////////////////////
/*	printf("Last Terminal ID : ");
	scanf("%u", &LastTermNum);
	if(LastTermNum <= 0 || LastTermNum > 63){
		printf(">>>> %u\n", LastTermNum);
		return;
	}
    LogPrintf("Last Terminal ID : %u\n", LastTermNum);*/

	LastTermNum = LAST_TERM_ADR;

#if (HLS_COMM==1)
	/////////////////////////////////////////////////
    // DLLをロード、API関数のエントリを取得する
    /////////////////////////////////////////////////
    if( !LoadSnDll(pcFileName) ) {
        // DLLのロードまたはEntry取得に失敗した場合終了
        LogPrintf("Program Terminated with Error!! (hit any key.)\n");
		printf("\a");
		HitAnyKeyWait();
		return;
    }

    /////////////////////////////////////////////////
    // すべてのAPI操作に先立ってOpenVxDを発行
    /////////////////////////////////////////////////
    err = OpenVxD();
    if(err != SNMA_NO_ERROR){
        LogPrintf("Failed to Excute <OpenVxD>  err = %08lx\n",err);
        ErrorExit(err);             // エラー終了
    } else {
        VxDOpen = TRUE;
    }

    /////////////////////////////////////////////////
    // 下位ドライバ（SYS/VxD）バージョンの取得
    /////////////////////////////////////////////////
    err = GETversion(&version);
    if(err == SNMA_NO_ERROR) {
		LogPrintf("%s.SYS/VxD Version = %2X.%02X\n", pcFileName, (version & 0xff00)>> 8, version & 0x00ff);
    } else {
        LogPrintf("Failed to Get %s.SYS/VxD Version!! (%08X)\n", pcFileName, err);
        ErrorExit(err);             // エラー終了
    }

    /////////////////////////////////////////////////
    // SN-100X-PCIMAボードの検出
    /////////////////////////////////////////////////
    err = BOARDlist(&ListBuf, (LPINT *)&NumBoards);
    if(err == SNMA_INIT_ERROR){
        LogPrintf("Failed to Get %s Board List!!\n", pcBoardName);
        ErrorExit(err);             // エラー終了
    } else {
        LogPrintf("Number of %s Master Board : %02d\n", pcBoardName, (BYTE)NumBoards);
        ListPtr = (CfgInfo *)&ListBuf.Bd0Inf;           // 最初のボード情報のポインタをセット
        for(lpcnt = 0;lpcnt < NumBoards;lpcnt ++)
        {
/***
            printf(" -------------------\n");
            printf("  Board ID = %02x\n",ListPtr->BoardId);     // ボードID(DSW1の設定値)
            printf("    Base I/O Address0(LCFG)         : %04x\n", ListPtr->CfgAddr);
            printf("    Base I/O Address1(%s): %04x\n", pcBoardName, ListPtr->BdAddr); // SN-1002-PCIMAの実体
            printf("    IRQ Number                      : %02x\n", ListPtr->IrqNum); // 割当てられたIRQ
            printf("    Data Transfer Rate              : %-2dMbps\n", (1 << ListPtr->XferRate) * 3);
                                                                // 現在のデータ伝送レート設定値
            printf("    Data Transfer Mode              : ");   // 現在のデータ伝送モード設定
            switch(ListPtr->XferMode)
            {
                case 0:
                    printf("Half Duplex\n");
                    break;
                case 1:
                    printf("Full Duplex\n");
                    break;
                default:
                    printf("Invalid\n");
            }
            printf("    IRQ Enable                      : ");
            switch(ListPtr->IrqEnb)                             // IRQ出力Enable設定
            {
                case 0:
                    printf("Disabled\n");
                    break;
                case 1:
                    printf("Enabled\n");
                    break;
                default:
                    printf("Invalid\n");
            }
            printf(" -------------------\n");
****/
        	ListPtr ++;                                 // 次のボード情報へのポインタを設定
        }
    }
    BoardId = ListBuf.Bd0Inf.BoardId;               // 最初に見つかったボードのボードID

	// 通信を停止する
	err = TERMstop(BoardId);
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Stop Communication!!\n");
		ErrorExit(err);                     // エラー終了
	}
	// ボードの初期化
	err = BOARDinit(BoardId);
	if(err == SNMA_INIT_ERROR){
		LogPrintf("Failed to Initialize SN-1002-PCIMA Board!!\n");
		ErrorExit(err);                     // エラー終了
	}

	/////////////////////////////////////////////////
	// 通信モード設定(伝送速度・伝送レート)
	/////////////////////////////////////////////////
	// SN-100X-PCIMAのドライバインストール時点での通信モード設定初期値は以下の通り。
	//  伝送速度 : 3Mbps
	//  伝送モード : Half Duplex
	//  ※システム起動毎に設定される初期設定値.
	//  ※初期設定値の変更は「コントロールパネル」の「SAVE NET」アプレットを起動して
	//    変更することができます。
	//  ※1998年11月現在時点では弊社標準製品のターミナル（モジュール）は全て3Mbps
	//    / Half Duplex使用となっている為、この部分の操作は必要ありません。
	//
	//  システム起動後に初期設定値から以下の方法で通信設定を変更することがで
	//  きます。
	//

	err = TERMsetup(BoardId, Xfer6M, HalfDup, EnbIrq);
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Setup Communication Mode!!\n");
		ErrorExit(err);                     // エラー終了
	}
//	LogPrintf("TERMsetup Xfer3M HalfDup EnbIrq\n");

	/////////////////////////////////////////////////
	// リトライ回数設定・取得
	/////////////////////////////////////////////////

	retrycnt = 1;                                   // 最大の設定値は7
	err = RETRYwrite(BoardId,(LPINT *)&retrycnt);   // リトライ回数書込み
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Set Retry Count!!\n");
		ErrorExit(err);             // エラー終了
	}
	err = RETRYread(BoardId,(LPINT *)&retrycnt);    // リトライ回数読込
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Retrieve Retry Count!!\n");
		ErrorExit(err);             // エラー終了
	}

	/////////////////////////////////////////////////
	// ターミナル・マップ情報 設定
	termmap[0] = Unused;		// マップ情報を未接続とする
	for(lpcnt = 1; lpcnt <= LastTermNum; lpcnt++) {
		termmap[lpcnt] = Io8;		// ターミナル・アドレス nを入出力に設定
	}
	for( ; lpcnt < 64; lpcnt ++) {
		termmap[lpcnt] = Unused;	// マップ情報を未接続とする
	}

	err = TERMwrite(BoardId, (LPINT *)&termmap[0]);  // ターミナル・マッピング情報書込み
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Store Terminal Mapping Information!!\n");
		ErrorExit(err);             // エラー終了
	}
	err = EEPROMwrite(BoardId);         // EEPROMにデータを書き込む
										// SN-1002-PCIMAの場合実際にはレジストリに登録。
										// この関数が実行されないとTERMwrite()による
										// データ設定は実際の動作に反映されない。
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Store EEPROM Information!!\n");
		ErrorExit(err);             // エラー終了
	}

	//printf("SAVENET通信を開始します");

	Sleep_Cnt(300);
    err = TERMstart(BoardId);               // ボードの通信を開始する
    if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Start Communication!!\n");
		ErrorExit(err);             // エラー終了
    }

	WaitProgressDot(200, 10);
	printf("\n\n");
#endif

	//SPxxxxAdjMain();		//検査メインループ
	SPxxxxMain();

#if (HLS_COMM==1)
    err = TERMstop(BoardId);			// ボードの通信を停止する
    if(err != SNMA_NO_ERROR){
        LogPrintf("Failed to Stop Communication!!\n");
		ErrorExit(err);					// エラー終了
    }

	CloseVxD();                         // 終了前にCloseVxDを必ず発行
	VxDOpen = FALSE;

    FreeLibrary(hDll);                  // DLLを開放する
#endif
	
	if( m_hSpComHandle != INVALID_HANDLE_VALUE ){
		CloseHandle(m_hSpComHandle);
		m_hSpComHandle = INVALID_HANDLE_VALUE;
	}
	
	AioUnitClose();		//CONTEC AIOユニット用ドライバをCLOSEする
	
	printf("\nHit any key.\n");
	HitAnyKeyWait();
}

//////////////////////
//SAVENET 初期出力
//検査基板側(#4～#12),検査機側(#48～#58)のDO出力をCLRする
//※ #55のリレー制御ユニットもCLRする場合は、引数bAllUnitにTRUEを指定する
void SnOutputInit(BOOL bAllUnit)
{
	WORD	ii;

	//検査基板側の出力をCLR
	for( ii=SP_XXXX_SW1_START_ADR; ii<=SP_XXXX_SW1_START_ADR+8; ii++ ){
		DATAout(BoardId, ii, 0x0000);
	}

	//検査機側の出力をCLR
	for( ii=ADR_DO_START; ii<=ADR_DO_END; ii++ ){
		if( ii == SN_ADR_RELAY_OUT_UNIT ){	//リレー制御ユニット(#55)
			if( bAllUnit == TRUE ){
				DATAout(BoardId, ii, 0x0000);
				m_snTestCtrlBit.WORD = 0x0000;
			}
		}
		else{
			DATAout(BoardId, ii, 0x0000);
		}
	}
}

///////////////////////
//検査基板名をiniファイルから取得する
void SPxxxxGetBoardName()
{
	if( m_nIniBoardID == 0 ){
		m_nSPxxxxBoardID = SP_BID_UPPER_IF_BOARD;
		if( m_nIniSpUpperBoardType == 0 ){
			strcpy(m_cSPxxxxBoardName, "SP_UPPER_IF_BOARD");
		}
		else{
			strcpy(m_cSPxxxxBoardName, "SP_UPPER_IF_BOARD2");
		}
	}
	else if( m_nIniBoardID == 1 ){
		m_nSPxxxxBoardID = SP_BID_FRONT_IF_BOARD;
		if( m_nIniSpFrontBoardType == 0 ){
			strcpy(m_cSPxxxxBoardName, "SP_FRONT_IF_BOARD");
		}
		else{
			strcpy(m_cSPxxxxBoardName, "SP_FRONT_IF_BOARD2");
		}
	}
	else if( m_nIniBoardID == 2 ){
		m_nSPxxxxBoardID = SP_BID_MOTHER_BOARD;
		if( m_nIniSpMotherBoardType == 0 ){
			strcpy(m_cSPxxxxBoardName, "SP_MOTHER_BOARD");
		}
		else{
			strcpy(m_cSPxxxxBoardName, "SPIN_MOTHER_BOARD");
		}
	}
	else{
		m_nSPxxxxBoardID = -1;
		strcpy(m_cSPxxxxBoardName, "SP_XXXX_BOARD");
	}
}

////////////////////////////////////////////////////////////////////
//			SP_XXXX_BOARD 調整・検査メイン
//			ピン治具による調整検査
////////////////////////////////////////////////////////////////////
void	SPxxxxMain()
{
	int	nInputVal = -1;

	while(ForeverLoop){
		SPxxxxGetBoardName();	//検査基板名取得
		SnOutputInit(TRUE);		//SAVENET 初期出力全CLR

		system("cls");			//画面クリア
		printf("-------------------------------------------------------\n");
		printf(" <<<  %s 検査用ソフト (Software Ver.%s)   >>>\n", m_cSPxxxxBoardName, SOFTWARE_VERSION);
		printf("-------------------------------------------------------\n");
		printf("<<<<<<<<<<<<<<<<   検査項目選択メニュー  >>>>>>>>>>>>>>>>\n");
		printf("[1]:全検査開始\n");
		printf("[2]:個別検査メニュー\n");
		printf("[Esc]:検査ソフト終了\n");
		printf("\n");

		printf("\n検査機の電源を投入後、メニューを選択してください-->");
	
		//キー入力待ち
		nInputVal = KeyInputWait();
		if( nInputVal == 0x1B ){
			break;		//メインメニューに戻る
		}
		else{
			printf("%c\n", nInputVal);
			//入力キーの確認
			switch(nInputVal){
			case (int)'1':	//全検査開始
				SPxxxxAllTestStart();
				break;
			case (int)'2':	//個別検査メニュー
				SPxxxxTestSelectMenu();
				break;
			default:
				break;
			}
		}
	}
}

//全検査開始
void SPxxxxAllTestStart()
{
	BOOL	berr;
	system("cls");		//画面クリア
	berr = SPxxxxSequenceTest('1', 'Z');

	//system("cls");		//画面クリア
	printf("\n\n\a");
	printf("_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/\n");
	if( berr == TRUE ){
		printf("  [検査結果：OK] 何かキーを押してください。\n");
	}
	else{
		printf("  [検査結果：NG] 何かキーを押してください。\a\a\a\n");
	}
	printf("_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/\n");
	HitAnyKeyWait();
}

//////////////////////////////////////////////////////////
//SPxxxx基板のパラメータで指示された検査を順に実施する
BOOL SPxxxxSequenceTest(int nInsStartID, int nInsEndID)
{
	int	ii;
	int nInputVal=-1;
	BOOL bErr = TRUE;
	BOOL bReturn = TRUE;

	//選択された検査項目の検査を順に実行する('1'→'9', 'A'→'Z')
	for(ii=nInsStartID; ii<=nInsEndID; ii++){
		if( (ii > '9') && (ii < 'A') ){
			ii = 'A';
		}
		system("cls");		//画面クリア
	
		//検査前の初期リレー出力（基板電源ON、CPLD(CN101側)に切り替え、TST4端子出力、E8A接続断、HLS接続）
		SnRelayOutputAllBit(1, 0, 0, 0, 1);

		switch(ii){
		case (int)'1':		//電源電圧確認
			bErr = SPxxxxDC_PwrChk(ii);
			break;
		case (int)'2':		//ファームウェア書き込み
			bErr = SPxxxxFirmWrite(ii);
			break;
		case (int)'3':		//検査用CPLD書き込み(SPIN,SP_MOTHERのみ)
			if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
				bErr = SPxxxxCPLDWrite(ii, 0);
			}
			break;
		case (int)'4':		//LED・消費電流目視確認
			bErr = SPxxxxLedTest(ii);
			break;
		case (int)'5':		//ボードID設定
			RS_SpComOpen();		//RS232C通信ポートをオープンする
			PcbPwrReBoot(TRUE);		//検査基板を再起動する
			bErr = SPxxxxBIDSetting(ii);
			break;
		case (int)'6':		//HLS通信テスト(NET1コネクタ側)
			bErr = SPxxxxComChk(ii, 0);
			break;
		case (int)'7':		//デジタル入出力複合テスト（機種毎に個別処理）
			if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
				if( m_nIniSpMotherBoardType == 0 ){
						//SP_MOTHER
				}
				else{	//SPIN_MOTHER
					bErr = SPxxxxDioChkOtherSpinMother(ii);
				}
			}
			else{
				//SP_FRONT_IF_BOARD
				//SP_UPPER_IF_BOARD
			}
			break;
		case (int)'8':		//デジタル入出力折り返しテスト(Auto)
			bErr = SPxxxxDioChkAuto(ii, TRUE, TRUE);
			break;
		case (int)'A':		//アナログ入力調整
			if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ||
				m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ||
				m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
				RS_SpComOpen();		//RS232C通信ポートをオープンする
				PcbPwrReBoot(TRUE);		//検査基板を再起動する
				bErr = SPxxxxAdInAdj(ii, -1, -1);
			}
			break;
		case (int)'B':		//アナログ出力調整
			if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
				RS_SpComOpen();		//RS232C通信ポートをオープンする
				PcbPwrReBoot(TRUE);		//検査基板を再起動する
				//bErr = SPxxxxDaOutAdj		//SPIN_MOTHERでは使用しないので後まわし
			}
			break;
		case (int)'C':		//アナログ入力精度確認
			if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ||
				m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ||
				m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
				bErr = SPxxxxAdinChk(ii, -1, -1);
			}
			break;
		case (int)'D':		//アナログ出力精度確認
			if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
				//SPxxxxDaoutChk	//SPIN_MOTHERでは使用しないので後まわし
			}
			break;

		case (int)'E':		//製品用CPLD書き込み(SPIN,SP_MOTHERのみ)
			if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
				bErr = SPxxxxCPLDWrite(ii, 1);
			}
			break;
		case (int)'F':		//HLS通信テスト(NET2コネクタ側)
			bErr = SPxxxxComChk(ii, 1);
			break;

		default:
			break;
		}

		if( bErr != TRUE ){
			bReturn = FALSE;
			if( ii == nInsEndID ){
				//最終検査のとき
				break;
			}
			else{
				//検査途中のとき
				printf("検査を中止しますか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					printf("\n--------------------------");
					printf("\n---検査を中止しました！---");
					printf("\n--------------------------\a\n\n");
					break;
				}
			}
		}

		//RS232C通信ポートをCLOSE後、検査基板を再起動する
		if( m_hSpComHandle != INVALID_HANDLE_VALUE ){
			CloseHandle(m_hSpComHandle);
			m_hSpComHandle = INVALID_HANDLE_VALUE;
			PcbPwrReBoot(FALSE);		//検査基板を再起動する
		}
	}	//for end

	return bReturn;
}

//////////////////////////////////////////
//個別検査メニュー
#define	PCB_TEST_LAST_CODE		('F')
void SPxxxxTestSelectMenu()
{
	int	nInputVal = -1;
	BOOL bInspectSkip=FALSE;
	int	nInsStartID = -1;
	int	nInsEndID = -1;
	BOOL bErr = TRUE;
	//int	ii;

	while(ForeverLoop){
		bInspectSkip = FALSE;
		bErr = TRUE;
		SnOutputInit(FALSE);		//SAVENET 初期出力

		system("cls");		//画面クリア
		printf("<<<<<<<<<<<  個別検査メニュー  >>>>>>>>>\n");
		printf("[1]:電源電圧確認\n");
		printf("[2]:ファームウェア書き込み\n");
		if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[3]:検査用CPLD書き込み\n");
		}
		printf("[4]:LED点灯／消費電流確認\n");
		printf("[5]:ボードID設定\n");
		printf("[6]:HLS通信テスト(NET1コネクタ側)\n");

		if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[7]:デジタル入出力複合テスト\n");
		}
		printf("[8]:デジタル入出力折り返しテスト\n");
		
		//printf("\n");

		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ||
			m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ||
			m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[A]:アナログ入力調整\n");
		}

		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
			printf("[B]:アナログ出力調整\n");
		}

		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ||
			m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ||
			m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[C]:アナログ入力精度確認\n");
		}

		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
			printf("[D]:アナログ出力精度確認\n");
		}

		if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[E]:検査用CPLD書き込み\n");
		}
		printf("[F]:HLS通信テスト(NET2コネクタ側)\n");
		printf("\n[Esc]:メインメニューに戻る\n");

		printf("\nメニューを選択してください-->");

		//キー入力待ち
		nInputVal = KeyInputWait();
		if( nInputVal == 0x1B ){
			break;		//個別検査終了
		}
		else{
			if( nInputVal >= (int)'a' && nInputVal <= (int)'z' ){
				nInputVal = nInputVal - 0x20;		//小文字アルファベットは大文字に変換して処理する
			}
			printf("%c\n", nInputVal);
			//入力キーの確認
			switch(nInputVal){
			case (int)'1':		//電源電圧確認
			case (int)'2':		//ファームウェア書き込み
			case (int)'3':		//検査用CPLD書き込み(SPIN,SP_MOTHERのみ)
			case (int)'4':		//LED点灯／消費電流確認
			case (int)'5':		//ボードID設定
			case (int)'6':		//HLS通信テスト(NET1コネクタ側)
			case (int)'7':		//デジタル入出力複合テスト
			case (int)'8':		//デジタル入出力折り返しテスト
			case (int)'A':		//アナログ入力調整
			case (int)'B':		//アナログ出力調整
			case (int)'C':		//アナログ入力精度確認
			case (int)'D':		//アナログ出力精度確認
			case (int)'E':		//製品用CPLD書き込み(SPIN,SP_MOTHERのみ)
			case (int)'F':		//HLS通信テスト(NET2コネクタ側)
				nInsStartID = nInputVal;
				nInsEndID = nInputVal;
				break;
			default:
				nInsStartID = -1;
				nInsEndID = -1;
				bInspectSkip = TRUE;		//検査なし
				break;
			}

			if( bInspectSkip != TRUE ){
				if( nInsStartID != PCB_TEST_LAST_CODE ){
					printf("[%c]以降も続けて、検査しますか？(y or n) -->", nInsStartID);
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						nInsEndID = PCB_TEST_LAST_CODE;
					}
				}

				bErr = SPxxxxSequenceTest(nInsStartID, nInsEndID);	//個別検査を開始する
				//system("cls");		//画面クリア
				printf("\n\n\a");
				printf("_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/\n");
				if( bErr == TRUE ){
					printf("  [検査結果：OK] 何かキーを押してください。\n");
				}
				else{
					printf("  [検査結果：NG] 何かキーを押してください。\a\a\a\n");
				}
				printf("_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/\n");
				HitAnyKeyWait();
			}
		}
	}	//while end
}

/////////////////////////////////////
//基板電源・CPLD選択・E8A接続/切断・HLS通信ライン接続/断線切り替え(リレー制御)
#define	BIT_SEL_PCBPWR_CTRL		(8)
#define	BIT_SEL_CPLDSEL_CTRL	(9)
#define	BIT_SEL_TST4_CTRL		(13)
#define	BIT_SEL_E8ACON_CTRL		(14)
#define	BIT_SEL_HLSCON_CTRL		(15)

#define	CPLD_CONNECTOR_SEL_CN101	(0)
#define	CPLD_CONNECTOR_SEL_CN99		(1)
void SnRelayOutputBit(int nBitSelect, WORD wData)
{
	DWORD	err;

	if( BoardId != 0xffff ){
		switch(nBitSelect){
		case BIT_SEL_PCBPWR_CTRL:
			m_snTestCtrlBit.BIT.B08_PCB_PWR = wData;
			break;
		case BIT_SEL_CPLDSEL_CTRL:
			m_snTestCtrlBit.BIT.B09_CPLD_SEL = wData;
			break;
		case BIT_SEL_TST4_CTRL:
			m_snTestCtrlBit.BIT.B13_TST4 = wData;
			break;
		case BIT_SEL_E8ACON_CTRL:
			m_snTestCtrlBit.BIT.B14_E8A_CONNECT = wData;
			break;
		case BIT_SEL_HLSCON_CTRL:
			m_snTestCtrlBit.BIT.B15_HLS_CONNECT = wData;
			break;
		default:
			return;
		}

		err = DATAout(BoardId, SN_ADR_RELAY_OUT_UNIT, (WORD)m_snTestCtrlBit.WORD);
		if( err != SNMA_NO_ERROR ){
			printf("DATAout() err=%ld\n", err);
			printf("何かキーを押してください。\a\n");
			HitAnyKeyWait();
		}
		else{
			Sleep_Cnt(100);
		}
	}
}
//0：該当bit OFF、1：該当bit ON、0/1以外：該当bitの切替なし
void SnRelayOutputAllBit(WORD wPcbPwr, WORD wCpldSel, WORD wTst4En, WORD wE8aCon, WORD wHlsCon)
{
	DWORD	err;

	if( BoardId != 0xffff ){
		if( wPcbPwr <= 1 ){ 
			m_snTestCtrlBit.BIT.B08_PCB_PWR = wPcbPwr;
		}
		if( wCpldSel <= 1 ){ 
			m_snTestCtrlBit.BIT.B09_CPLD_SEL = wCpldSel;
		}
		if( wTst4En <= 1 ){
			m_snTestCtrlBit.BIT.B13_TST4 = wTst4En;
		}
		if( wE8aCon <= 1 ){ 
			m_snTestCtrlBit.BIT.B14_E8A_CONNECT = wE8aCon;
		}
		if( wHlsCon <= 1 ){ 
			m_snTestCtrlBit.BIT.B15_HLS_CONNECT = wHlsCon;
		}
		err = DATAout(BoardId, SN_ADR_RELAY_OUT_UNIT, (WORD)m_snTestCtrlBit.WORD);
		if( err != SNMA_NO_ERROR ){
			printf("DATAout() err=%ld\n", err);
			printf("何かキーを押してください。\a\n");
			HitAnyKeyWait();
		}
		else{
			Sleep_Cnt(100);
		}
	}	
}

////////////////////////////
//検査基板の電源再投入
// bRsModeにTRUE指定時は、TST4端子をON（RS-232C通信モード）でMPUを起動する
void PcbPwrReBoot(BOOL bRsMode)
{
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 0);	//電源OFF
	if( bRsMode == TRUE ){
		//基板電源OFF、CPLD(CN101側)[切替なし]、TST4端子出力ON、E8A接続[切替なし]、HLS接続[切替なし]
		SnRelayOutputAllBit(0, 0xffff, 1, 0xffff, 0xffff);
	}
	else{
		//基板電源OFF、CPLD(CN101側)[切替なし]、TST4端子出力OFF、E8A接続[切替なし]、HLS接続[切替なし]
		SnRelayOutputAllBit(0, 0xffff, 0, 0xffff, 0xffff);
	}
	Sleep_Cnt(500);	//少し待つ
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 1);	//電源ON
	Sleep_Cnt(500);	//少し待つ
}

/////////////////////////////////////
//アナログ入出力ch切り替え(リレー制御)
void	SnMultiMeterMeasureSel(WORD wANAch, int nInOutSel, int nVISel, int nLoadSel)
{
	DWORD	err;

	if( BoardId != 0xffff ){
		m_snTestCtrlBit.BIT.B00B03_ANALOG_CH = wANAch;	//アナログch番号を指定(ch4:DC5V, ch5:DC24V)
		if( nVISel == ANA_VI_SEL_VOLT ){
			//電圧入出力モード
			m_snTestCtrlBit.BIT.B04_ANALOG_MODE_IN = 1;		//マルチメータ接続を電圧測定用に切り替える
			m_snTestCtrlBit.BIT.B05_ANALOG_MODE_OUT = 1;	//D/A出力を電圧出力モードに切り替える
		}
		else{
			//電流入出力モード
			m_snTestCtrlBit.BIT.B04_ANALOG_MODE_IN = 0;		//マルチメータ接続を電流測定用に切り替える
			if( nInOutSel == ANA_INOUT_SEL_OUT ){		//D/A出力検査モード時
				//SPxxxx基板では電流出力は無いので、ここに入ることは無い
				m_snTestCtrlBit.BIT.B05_ANALOG_MODE_OUT = 0;	//D/A出力を電流出力モードに切り替える
			}
			else{										//A/D入力検査モード時
				m_snTestCtrlBit.BIT.B05_ANALOG_MODE_OUT = 1;	//D/A出力を電圧出力モードに切り替える
			}
		}

		//抵抗負荷の有無
		if( nLoadSel == ANA_LOAD_SEL_OPEN ){
			m_snTestCtrlBit.BIT.B06_ANALOG_RES_VAL = 0;		//電圧出力の抵抗負荷はOPENにする
		}
		else{
			m_snTestCtrlBit.BIT.B06_ANALOG_RES_VAL = 1;		//電圧出力の抵抗負荷を有効にする(電流出力:250Ω、電圧出力:1KΩ)
		}

		//アナログ入力orアナログ出力
		if( nInOutSel == ANA_INOUT_SEL_OUT ){		//D/A出力検査モード時
			m_snTestCtrlBit.BIT.B07_ANALOG_IO_SL = 0;		//マルチメータを検査機内部のアナログ出力検査部側に接続する
		}
		else{
			m_snTestCtrlBit.BIT.B07_ANALOG_IO_SL = 1;		//マルチメータを検査機内部のアナログ入力検査部側に接続する
		}

		err = DATAout(BoardId, SN_ADR_RELAY_OUT_UNIT, (WORD)m_snTestCtrlBit.WORD);
		if( err != SNMA_NO_ERROR ){
			printf("DATAout() err=%ld\n", err);
			printf("何かキーを押してください。\a\n");
			HitAnyKeyWait();
		}
		else{
			Sleep_Cnt(100);
		}
	}
}

//////////////////////////////////////////
//デジタルマルチメータを使用して検査基板の電源電圧範囲のチェックを行う
#define		RATE5PER					(5.0)
#define		RATE3PER					(3.0)
#define		RATE05PER					(0.5)
//#define		RATE_D12PLUSPER				(10.0)	//D12Vのプラス側許容精度
#define		DC_PWR_RANGE_CHK_NUM		(2)
typedef struct{
	char	cSignalName[8];
	WORD	wChSel;	//ch4:DC5V, ch5:DC24V
	double	fStdV;
	double	fRatePer;
	double	fRangeMinV;
	double	fRangeMaxV;
}DC_PWR_CHK_TABLE;
DC_PWR_CHK_TABLE	m_DcPwrChkTable[DC_PWR_RANGE_CHK_NUM+1]={
	{"NC",		0,	0.0,	0.0,		0.0,	0.0},
	{"24V",		5,	24.0,	RATE5PER,	0.0,	0.0},
	{"5V",		4,	5.0,	RATE5PER,	0.0,	0.0},
};

#define	SN_MM_SEL_NC				(0)
#define	SN_MM_SEL_D24V				(1)
#define	SN_MM_SEL_D5V				(2)
BOOL	SPxxxxDC_PwrChk(int nInsID)
{
	BOOL	bRet=TRUE;
	int		ii;
	int		nInputVal;
	HANDLE	forMultiMaterHandles = NULL;
	BOOL	retStart_HP34401A=0;
	BOOL	retChangeVA_DC_HP34401A;
	double	fVoltageVal;

	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);

	lstrcpy(m_strLogFileName, "SPxxxx_DCChk.log");
	LogPrintfCsv(m_strLogFileName, 
		"\n■■■ %c:DC電源電圧確認 %s %s START ■■■\n", nInsID, strDate, strTime);

	//デジタルマルチメータの準備
	retStart_HP34401A = Start_HP34401A(
		m_cComPort,
		&forMultiMaterHandles	//通信リソースのハンドルを返すアドレス
	);
	if( ! retStart_HP34401A ){
		printf("MultiMeter 34401A Initialize Error!!\n");
		return FALSE;
	}

	//マルチメータのレンジを電圧入力に設定する
	retChangeVA_DC_HP34401A = ChangeVoltageDC_HP34401A(
		forMultiMaterHandles	//通信リソースハンドル
	);
	if( !retChangeVA_DC_HP34401A ){
		printf("MultiMeter 34401A Voltage Setting Error!!\n");
		End_HP34401A(forMultiMaterHandles);	//通信リソースのハンドルを閉じる
		return FALSE;
	}

	if( bRet == TRUE ){
		while(ForeverLoop){
			for( ii=1; ii<=DC_PWR_RANGE_CHK_NUM; ii++ ){
				m_DcPwrChkTable[ii].fRangeMinV = m_DcPwrChkTable[ii].fStdV - 
					( m_DcPwrChkTable[ii].fStdV * (m_DcPwrChkTable[ii].fRatePer/100.0));

					m_DcPwrChkTable[ii].fRangeMaxV = m_DcPwrChkTable[ii].fStdV + 
						( m_DcPwrChkTable[ii].fStdV * (m_DcPwrChkTable[ii].fRatePer/100.0));

				//測定電圧用リレー(アナログch)を切り替える
				SnMultiMeterMeasureSel(m_DcPwrChkTable[ii].wChSel, ANA_INOUT_SEL_OUT, ANA_VI_SEL_VOLT, ANA_LOAD_SEL_OPEN);
			
				//測定開始（マルチメータの場合）
				if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
					printf("[NG] 電圧値を取得できませんでした。何かキーを押してください。\a\n");
					HitAnyKeyWait();
					bRet = FALSE;
					break;
				}
				else{
					if( (fVoltageVal < m_DcPwrChkTable[ii].fRangeMinV) ||
						(fVoltageVal > m_DcPwrChkTable[ii].fRangeMaxV) ){
						//電圧値異常
						LogPrintfCsv(m_strLogFileName, 
							"[NG] %8s: [%6.3f]V, [正常範囲(±%3.1f%%/FS)=%6.3fV～%6.3fV]\n", 
							m_DcPwrChkTable[ii].cSignalName, fVoltageVal,
							m_DcPwrChkTable[ii].fRatePer,
							m_DcPwrChkTable[ii].fRangeMinV, m_DcPwrChkTable[ii].fRangeMaxV);
						bRet = FALSE;		//電圧値異常
					}
					else{
						//電圧値正常
						LogPrintfCsv(m_strLogFileName, 
							"[OK] %8s: [%6.3f]V, [正常範囲(±%3.1f%%/FS)=%6.3fV～%6.3fV]\n", 
							m_DcPwrChkTable[ii].cSignalName, fVoltageVal,
							m_DcPwrChkTable[ii].fRatePer,
							m_DcPwrChkTable[ii].fRangeMinV, m_DcPwrChkTable[ii].fRangeMaxV);
					}
				}
			}	//for end
			//測定電圧をNCに戻す
			SnMultiMeterMeasureSel(SN_MM_SEL_NC, ANA_INOUT_SEL_IN, ANA_VI_SEL_VOLT, ANA_LOAD_SEL_OPEN);

			if( bRet != TRUE ){
				printf("再検査しますか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					bRet = TRUE;
					LogPrintfCsv(m_strLogFileName, "\n----再検査----\n");
					continue;			//再検査実施
				}
			}
			break;
		}	//while(再検査用) end 
	}
	
	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"DC電源電圧確認結果：%s\n", ((bRet==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	//マルチメータ通信リソースのハンドルを閉じる
	if( forMultiMaterHandles != NULL ){
		End_HP34401A(forMultiMaterHandles);
	}

	/////////////////////////////////////////////////////////////////////
	//終了処理
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:DC電源電圧確認 %s %s FINISH ■■■\n", nInsID, strDate, strTime);

	return bRet;
}

//////////////////////////////////////////
//SP_XXXX基板にE8Aを使用してファームウェアを書き込む
// (F/Wの書き込みソフトは HEWを使用)
BOOL SPxxxxFirmWrite(int nInsID)
{
	BOOL	bRet=TRUE;
	int		nInputVal=-1;
	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);

	lstrcpy(m_strLogFileName, "SPxxxx_FirmWrite.log");
	LogPrintfCsv(m_strLogFileName, 
		"\n■■■ %c:ファームウェア書き込み %s %s START ■■■\n", nInsID, strDate, strTime);

	//検査品の電源をOFF
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 0);	//電源OFF
	Sleep_Cnt(500);	//少し待つ

	//E8A接続(リレー切り替え)
	SnRelayOutputBit(BIT_SEL_E8ACON_CTRL, 1);	//E8A接続

	//検査品の電源をON
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 1);	//電源ON
	
	//HEWを使用してファームウェアを書き込む
	printf("\n");
	printf("------------------------------------------------------\n");
	printf("  ＜CN100(ファームウェアの書き込み＞\n");
	printf("   HEW を使用してファームウェアの書き込みを開始してください。\n");
	printf("------------------------------------------------------\n");
	printf("\n");
	
	printf("終了する場合は、'Y'(正常終了)または、'N'(異常終了)を入力してください\n");
	while(ForeverLoop){
		nInputVal = KeyInputWait();
		if( nInputVal == 'y' || nInputVal == 'Y' || nInputVal == 0x0d ){
			//正常終了
			break;
		}
		else if( nInputVal == 'n' || nInputVal == 'N' || nInputVal == 0x1B ){
			//異常終了
			bRet = FALSE;
			break;
		}
		else{	//その他のキーは無視する
			continue;
		}
	}

	//LogPrintfCsv(m_strLogFileName,"  [%s]: CN100 ファームウェアの書き込み\n", ((bRet==TRUE) ? "OK":"NG"));
	
	//検査品の電源をOFF
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 0);
	Sleep_Cnt(500);	//少し待つ
	
	//E8A切断
	SnRelayOutputBit(BIT_SEL_E8ACON_CTRL, 0);
	
	//検査品の電源をON
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 1);

	
	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"ファームウェア書き込み：%s\n", ((bRet==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	/////////////////////////////////////////////////////////////////////
	//終了処理
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:ファームウェア書き込み %s %s FINISH ■■■\n", nInsID, strDate, strTime);

	return bRet;
}

//////////////////////////////////////////
//SP_MOTHER, SPIN_MOTHER用のCPLDを書き込む
// (CPLDの書き込みソフトは QuatusIIを使用)
//nCpldId=0: 検査用 CPLD書き込み
//nCpldId=1: 製品用 CPLD書き込み
BOOL SPxxxxCPLDWrite(int nInsID, int nCpldId)
{
	BOOL	bRet=TRUE;
	int		nInputVal=-1;
	int		ii;
	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);

	lstrcpy(m_strLogFileName, "SPxxxx_CpldWrite.log");
	LogPrintfCsv(m_strLogFileName, 
		"\n■■■ %c:CPLD書き込み %s %s START ■■■\n", nInsID, strDate, strTime);

	for(ii=0; ii<2; ii++){	//0:CN101(HLS)側, 1:CN99(検査・製品用ファーム)側
		if( ii == 0 ){
			//製品用CPLD書き込み時、SP_MOTHER_BOARDの検査時は
			//CN101(HLS CPLD側)の書き込みは必要なし
			if( (nCpldId != 0) ||
                (m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD && m_nIniSpMotherBoardType ==  0) ){
				continue;
			}
		}
		
		if( nCpldId == 0 ){	//検査用CPLD書き込み時
			printf("\n");
			printf("------------------------------------------------------\n");
			if( ii == 0 ){
				printf("  ＜CN101(HLS CPLD側)の書き込み＞\n");
			}
			else{
				printf("  ＜CN99(検査用CPLD)側の書き込み＞\n");
			}
		}
		else{				//製品用CPLD書き込み時
			printf("\n");
			printf("------------------------------------------------------\n");
			printf("  ＜CN99(製品用CPLD)側の書き込み＞\n");
		}

		printf("   Quartus II を使用してCPLDの書き込みを開始してください。\n");
		printf("------------------------------------------------------\n");
		printf("\n");
	
		//CPLD書き込み用JTAG接続の切り替え
		switch(ii){
		case 0:
			SnRelayOutputBit(BIT_SEL_CPLDSEL_CTRL, CPLD_CONNECTOR_SEL_CN101);
			break;
		case 1:
			SnRelayOutputBit(BIT_SEL_CPLDSEL_CTRL, CPLD_CONNECTOR_SEL_CN99);
			break;
		}

		printf("終了する場合は、'Y'(正常終了)または、'N'(異常終了)を入力してください\n");
		while(ForeverLoop){
			nInputVal = KeyInputWait();
			if( nInputVal == 'y' || nInputVal == 'Y' || nInputVal == 0x0d ){
				//正常終了
				break;
			}
			else if( nInputVal == 'n' || nInputVal == 'N' || nInputVal == 0x1B ){
				//異常終了
				bRet = FALSE;
				break;
			}
			else{	//その他のキーは無視する
				continue;
			}
		}	

		if( nCpldId == 0 ){	//検査用CPLD書き込み時
			if( ii == 0 ){
				LogPrintfCsv(m_strLogFileName,"  [%s]: CN101(HLS CPLD)側の書き込み\n", ((bRet==TRUE) ? "OK":"NG"));
			}
			else{
				LogPrintfCsv(m_strLogFileName,"  [%s]: CN99(検査用CPLD)側の書き込み\n", ((bRet==TRUE) ? "OK":"NG"));
			}
		}
		else{				//製品用CPLD書き込み時
			LogPrintfCsv(m_strLogFileName,"  [%s]: CN99(製品用CPLD)側の書き込み\n", ((bRet==TRUE) ? "OK":"NG"));
		}
	}

	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"CPLD書き込み：%s\n", ((bRet==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	/////////////////////////////////////////////////////////////////////
	//終了処理
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:CPLD書き込み %s %s FINISH ■■■\n", nInsID, strDate, strTime);

	return bRet;
}

//////////////////////////////////////////
//SP_XXXX基板の種別設定を行う
BOOL SPxxxxBIDSetting(int nInsID)
{
	int		ii;
	int		nInputVal;
	int		nNewBoardID;
	char	strDate[32];
	char	strTime[32];
	DWORD	dwAdOffsetTemp[SP_XXXX_ADCH_MAX];
	DWORD	dwAdGainTemp[SP_XXXX_ADCH_MAX];
	DWORD	dwDaOffsetTemp[SP_XXXX_DACH_MAX];
	DWORD	dwDaGainTemp[SP_XXXX_DACH_MAX];
	BOOL	bMatchChk;
	BOOL	bRet = FALSE;

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);

	lstrcpy(m_strLogFileName, "SPxxxx_Seting.log");
	LogPrintfCsv(m_strLogFileName, 
		"\n■■■ %c:Board ID Check %s %s START ■■■\n", nInsID, strDate, strTime);

	for( ii=0; ii<SP_XXXX_ADCH_MAX; ii++ ){
		dwAdOffsetTemp[ii] = 0xffff;
		dwAdGainTemp[ii] = 0xffff;
	}
	for( ii=0; ii<SP_XXXX_DACH_MAX; ii++ ){
		dwDaOffsetTemp[ii] = 0xffff;
		dwDaGainTemp[ii] = 0xffff;
	}

	RS_SpComOpen();
	
	if( m_hSpComHandle != INVALID_HANDLE_VALUE ){
		m_nSPxxxxBoardID = RS_SpUpdateBoardID(SP_BID_CHECK_ONLY, &m_nFirmVer);
		switch(m_nSPxxxxBoardID){
		case SP_BID_UPPER_IF_BOARD:
			if( m_nIniSpUpperBoardType == 0 ){
				strcpy(m_cSPxxxxBoardName, "SP_UPPER_IF_BOARD");
			}
			else{
				strcpy(m_cSPxxxxBoardName, "SP_UPPER_IF_BOARD2");
			}
			break;
		case SP_BID_FRONT_IF_BOARD:
			if( m_nIniSpFrontBoardType == 0 ){
				strcpy(m_cSPxxxxBoardName, "SP_FRONT_IF_BOARD");
			}
			else{
				strcpy(m_cSPxxxxBoardName, "SP_FRONT_IF_BOARD2");
			}
			break;
		case SP_BID_MOTHER_BOARD:
			if( m_nIniSpMotherBoardType == 0 ){
				strcpy(m_cSPxxxxBoardName, "SP_MOTHER_BOARD");
			}
			else{
				strcpy(m_cSPxxxxBoardName, "SPIN_MOTHER_BOARD");
			}
			break;
		default:
			strcpy(m_cSPxxxxBoardName, "Unknown Board Type.");
			break;
		}

		if( m_nSPxxxxBoardID != -1 ){
			LogPrintfCsv(m_strLogFileName, "ボード側で設定されているタイプは[%s]です。\n", m_cSPxxxxBoardName);

			if( RS_SpSetAdOffsetGain(dwAdOffsetTemp, dwAdGainTemp, dwAdOffsetTemp, dwAdGainTemp) == TRUE ){	//AD補正値の読出しのみ実行
				if( RS_SpSetDaOffsetGain(dwDaOffsetTemp, dwDaGainTemp, dwDaOffsetTemp, dwDaGainTemp) == TRUE ){	//DA補正値の読出しのみ実行
					switch(m_nSPxxxxBoardID){
					case SP_BID_UPPER_IF_BOARD:		//AD 6ch
						for(ii=0; ii<6; ii++){
							LogPrintfCsv(m_strLogFileName, " [A/D ch%d] Offset=0x%04X Gain=0x%04X\n", ii, dwAdOffsetTemp[ii], dwAdGainTemp[ii]);
						}
						for(ii=0; ii<SP_XXXX_DACH_MAX; ii++){
							LogPrintfCsv(m_strLogFileName, " [D/A ch%d] Offset=0x%04X Gain=0x%04X\n", ii, dwDaOffsetTemp[ii], dwDaGainTemp[ii]);
						}
						break;

					case SP_BID_FRONT_IF_BOARD:		//AD 4ch
						for(ii=0; ii<4; ii++){
							LogPrintfCsv(m_strLogFileName, " [A/D ch%d] Offset=0x%04X Gain=0x%04X\n", ii, dwAdOffsetTemp[ii], dwAdGainTemp[ii]);
						}
						break;

					case SP_BID_MOTHER_BOARD:		//AD 1ch
						for(ii=0; ii<1; ii++){
							LogPrintfCsv(m_strLogFileName, " [A/D ch%d] Offset=0x%04X Gain=0x%04X\n", ii, dwAdOffsetTemp[ii], dwAdGainTemp[ii]);
						}
						break;
					}
				}
				else{
					printf("D/A Offset/Gain設定値を確認できませんでした。\n");
				}
			}
			else{
				printf("A/D Offset/Gain設定値を確認できませんでした。\n");
			}
		}
		else{
			LogPrintfCsv(m_strLogFileName, "ボード側で設定されているタイプを確認できませんでした。\n");
		}

		//ボード側の設定とINIの設定が異なっているかを確認する
		bMatchChk = FALSE;
		if( m_nSPxxxxBoardID != -1 ){
			switch(m_nSPxxxxBoardID){
			case SP_BID_UPPER_IF_BOARD:
				if( m_nIniBoardID == 0 ){
					bMatchChk = TRUE;	//ボード側の設定とINIの設定は同じ
				}
				break;
			case SP_BID_FRONT_IF_BOARD:
				if( m_nIniBoardID == 1 ){
					bMatchChk = TRUE;	//ボード側の設定とINIの設定は同じ
				}
				break;
			case SP_BID_MOTHER_BOARD:
				if( m_nIniBoardID == 2 ){
					bMatchChk = TRUE;	//ボード側の設定とINIの設定は同じ
				}
				break;
			default:
				break;
			}
		}

		nNewBoardID = -1;
		//ボード側の設定とINIの設定が異なっているとき（INIの設定値をボードに設定するかを確認する）
		if( bMatchChk != TRUE ){
			switch(m_nIniBoardID){
			case 0:
				printf("ボードタイプ(Upper,Front,Mother)を\"Upper\"に設定しますか？(y or n) -->" );
				break;
			case 1:
				printf("ボードタイプ(Upper,Front,Mother)を\"Front\"に設定しますか？(y or n) -->" );
				break;
			case 2:
				printf("ボードタイプ(Upper,Front,Mother)を\"Mother\"に設定しますか？(y or n) -->" );
				break;
			default:
				bMatchChk = TRUE;
				break;
			}

			if( bMatchChk != TRUE ){
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					//INIの設定値をボードに設定する
					switch(m_nIniBoardID){
					case 0:
						nNewBoardID = SP_BID_UPPER_IF_BOARD;
						break;
					case 1:
						nNewBoardID = SP_BID_FRONT_IF_BOARD;
						break;
					case 2:
						nNewBoardID = SP_BID_MOTHER_BOARD;
						break;
					default:
						break;
					}
				}
			}
		}

		//ボードIDの変更を行うとき（Upper、Front、Motherを選択してもらう）
		if( nNewBoardID == -1 ){
			printf("ボードタイプの設定を変更しますか？(y or n) -->" );
			nInputVal = KeyInputWaitYorN();
			if( nInputVal == 'y' ){
				while(ForeverLoop){
					printf("\nボードを選択してください[0:Upper, 1:Front, 2:Mother] -->");
					nInputVal = KeyInputWait();
					printf("\n");
					if( nInputVal == 0x1B ){
						break;
					}
					else if( nInputVal == '0' || nInputVal == '1' || nInputVal == '2' ){
						switch(nInputVal){
						case '0':	//UPPER
							nNewBoardID = SP_BID_UPPER_IF_BOARD;
							break;
						case '1':	//FRONT
							nNewBoardID = SP_BID_FRONT_IF_BOARD;
							break;
						case '2':	//MOTHER
							nNewBoardID = SP_BID_MOTHER_BOARD;
							break;
						}
						break;
					}
				}
			}
			else{
				bRet = bMatchChk;
			}
		}

		//ボードIDをボード側に送信しEEPROMに記録する
		if( nNewBoardID != -1 ){
			nNewBoardID = RS_SpUpdateBoardID(nNewBoardID, &m_nFirmVer);
			if( nNewBoardID != -1 ){
				m_nSPxxxxBoardID = nNewBoardID;
				switch(nNewBoardID){
				case SP_BID_UPPER_IF_BOARD:
					if( m_nIniSpUpperBoardType == 0 ){
						strcpy(m_cSPxxxxBoardName, "SP_UPPER_IF_BOARD");
					}
					else{
						strcpy(m_cSPxxxxBoardName, "SP_UPPER_IF_BOARD2");
					}
					break;
				case SP_BID_FRONT_IF_BOARD:
					if( m_nIniSpFrontBoardType == 0 ){
						strcpy(m_cSPxxxxBoardName, "SP_FRONT_IF_BOARD");
					}
					else{
						strcpy(m_cSPxxxxBoardName, "SP_FRONT_IF_BOARD2");
					}
					break;
				case SP_BID_MOTHER_BOARD:
					if( m_nIniSpMotherBoardType == 0 ){
						strcpy(m_cSPxxxxBoardName, "SP_MOTHER_BOARD");
					}
					else{
						strcpy(m_cSPxxxxBoardName, "SPIN_MOTHER_BOARD");
					}
					break;
				default:
					strcpy(m_cSPxxxxBoardName, "Unknown Board Type.");
					break;
				}
				LogPrintfCsv(m_strLogFileName, "ボードIDを(%s)に設定しました。\n", m_cSPxxxxBoardName );
				bRet = TRUE;
			}
			else{
				printf("ボードIDの設定に失敗しました。\a\n" );
			}
		}
	}

	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"Board ID Check：%s\n", ((bRet==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:Board ID Check %s %s FINISH ■■■\n", nInsID, strDate, strTime);

	return TRUE;
}

///////////////////////////////////////////////////////
//通信検査(SP基板用)　nComID=0(NET1側):1(NET2側)
BOOL SPxxxxComChk(int nInsID, int nComID)
{
	int		ii, jj, kk;
	BOOL	bComChk=TRUE;
	BOOL	bRetCont=TRUE;
	HLS_CHK_PATTERN	*pTestPat;
	int	nTestPatNum;
	BOOL	bComStat;
	BOOL	bSwSetReqDisp;
	int		nInputVal;
	DWORD	dwStartTick;
	int		nDswMin, nDswMax;
	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);

	lstrcpy(m_strLogFileName, "SPxxxx_ComChk.log");
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:HLS通信チェック(NET%d) %s %s START ■■■\n", nInsID, nComID+1, strDate, strTime);
	if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
		pTestPat = m_HlsTestUpper;
		nTestPatNum = HLS_CHK_PATTERN_UPPER_NUM;
	}
	else if( m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ){
		pTestPat = m_HlsTestFront;
		nTestPatNum = HLS_CHK_PATTERN_FRONT_NUM;
	}
	else{
		pTestPat = m_HlsTestMother;
		nTestPatNum = HLS_CHK_PATTERN_MOTHER_NUM;
	}

	//if( m_nIniSpHlsTestDswAllOff != 1 ){
	//	//DSW設定を検査用のアドレスで終了する場合
	//	nTestPatNum--;
	//}

	for( ii=0; ii<nTestPatNum; ii++ ){
		if( ii == 0 ){
			if( nComID == 1 ){	//NET1とNET2の検査は最初と最後の2回実施するようにした
				//ii = (nTestPatNum-2);	//強制的にNET2側の検査に移行する
				ii = 2;	//強制的にNET2側の検査に移行する
			}
			else{
				printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				printf("NET1コネクタ側にHLS通信ケーブルを接続してください\n");
				printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				nInputVal = KeyInputWait();
				if ( nInputVal == 0x1B ){
					bRetCont = FALSE;
					break;
				}
			}
		}
		//if( ii == (nTestPatNum-2) ){
		if( ii == 2 ){
			if( nComID == 0 ) break;	//NET1とNET2の検査は最初と最後の2回実施するようにした
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			printf("NET2コネクタ側にHLS通信ケーブルを接続してください\n");
			printf("準備ができたら何かキーを押してください。\n");
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			nInputVal = KeyInputWait();
			if ( nInputVal == 0x1B ){
				bRetCont = FALSE;
				break;
			}
		}
		bSwSetReqDisp = FALSE;
		for( kk=0; kk<3; kk++ ){	//DSWが正しくセットされたかを（手操作のため）３回チェックする
			while(bRetCont == TRUE){
				bComStat = TRUE;	//指定範囲アドレスの通信・非通信状態チェック結果
				for(jj=0; jj<8; jj++){
					if( (pTestPat[ii].nComStartAdr[jj] != -1) && (pTestPat[ii].nComEndAdr[jj] != -1) ){
						if( pTestPat[ii].nSwTestCode[jj] <= 0 ){
							bComChk = FALSE;	//非通信状態であることをチェックする
						}
						else{
							bComChk = TRUE;		//通信状態であることをチェックする
						}
						if( SpComChk(pTestPat[ii].nComStartAdr[jj], pTestPat[ii].nComEndAdr[jj], bComChk) != TRUE ){
							bComStat = FALSE;
						}
					}
				}
				if( bComStat == TRUE ){
					break;	//全SWが正しくセットされたことを検出
				}
				else if(bSwSetReqDisp == FALSE){
					bSwSetReqDisp = TRUE;
					printf("\nHLS通信テスト[%d/%d]\n", ii+1, nTestPatNum);

					nDswMin = 999;
					nDswMax = 0;
					for(jj=0; jj<8; jj++){
						if( pTestPat[ii].nSwTestCode[jj] != -1 ){
							if( nDswMin > jj ){
								nDswMin = jj;
							}
							if( nDswMax < jj ){
								nDswMax = jj;
							}
							if( jj==2 ){	//UpperのSW3入力テスト
								if(pTestPat[ii].nSwTestCode[jj] == 0){
									printf("SW%d:%2d(0,0,0,0,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
									//printf("SW%dを%d(全OFF)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								}
								else if( pTestPat[ii].nSwTestCode[jj] == 12 ){
									printf("SW%d:%2d(0,0,1,1,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
									//printf("SW%dを%d(1:OFF, 2:OFF, 3:ON, 4:ON, 5:OFF, 6:OFF)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								}
								else if( pTestPat[ii].nSwTestCode[jj] == 21 ){
									printf("SW%d:%2d(1,0,1,0,1,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
									//printf("SW%dを%d(1:ON, 2:OFF, 3:ON, 4:OFF, 5:ON, 6:OFF)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								}
								else if( pTestPat[ii].nSwTestCode[jj] == 42 ){
									printf("SW%d:%2d(0,1,0,1,0,1)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
									//printf("SW%dを%d(1:OFF, 2:ON, 3:OFF, 4:ON, 5:OFF, 6:ON)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								}
								else{
									printf("SW%dを%2d(開始Adr=%d, 終了Adr=%d)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj],
											pTestPat[ii].nComStartAdr[jj], pTestPat[ii].nComEndAdr[jj]);
								}
							}
							else if(pTestPat[ii].nSwTestCode[jj] == 0){
								printf("SW%d:%2d(0,0,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%dを%d(全OFF)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else if( pTestPat[ii].nSwTestCode[jj] == 5 ){
								printf("SW%d:%2d(1,0,1,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%dを%d(1:ON, 2:OFF, 3:ON, 4:OFF)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else if( pTestPat[ii].nSwTestCode[jj] == 10 ){
								printf("SW%d:%2d(0,1,0,1)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%dを%d(1:OFF, 2:ON, 3:OFF, 4:ON)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else if( pTestPat[ii].nSwTestCode[jj] == 1 ){
								printf("SW%d:%2d(1,0,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%dを%d(1:ON, 2:OFF, 3:OFF, 4:OFF)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else if( pTestPat[ii].nSwTestCode[jj] == 2 ){
								printf("SW%d:%2d(0,1,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%dを%d(1:OFF, 2:ON, 3:OFF, 4:OFF)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else{
								printf("SW%dを%2d(開始Adr=%d, 終了Adr=%d)に設定してください\n", jj+1, pTestPat[ii].nSwTestCode[jj],
										pTestPat[ii].nComStartAdr[jj], pTestPat[ii].nComEndAdr[jj]);
							}

						}
					}
					printf("HLSアドレス設定用DSW%d～DSW%dを上記の設定にしてください。\n", nDswMin+1, nDswMax+1);
					printf("(Escキー:テストCancel)\n");
				}
				if( _kbhit() ){
					nInputVal = _getch();
					if( nInputVal == 0x1B ){
						bRetCont = FALSE;
						printf("\a通信テストがキャンセルされました\a\n");
						break;
					}
				}
			}
			if( bRetCont == TRUE ){
				Sleep_Cnt(200);
			}
		}

		if( bRetCont == TRUE ){
			printf("\n");
			dwStartTick = GetTickCount();
			do{
				bComStat = TRUE;	//指定範囲アドレスの通信・非通信状態チェック結果
				for(jj=0; jj<8; jj++){
					if( pTestPat[ii].nSwTestCode[jj] != -1 ){
						if( pTestPat[ii].nSwTestCode[jj] == 0 ){
							bComChk = FALSE;	//非通信状態であることをチェックする
						}
						else{
							bComChk = TRUE;		//通信状態であることをチェックする
						}
						if( SpComChk(pTestPat[ii].nComStartAdr[jj], pTestPat[ii].nComEndAdr[jj], bComChk) != TRUE ){
							printf("\n");
							if( bComChk == TRUE ){
								LogPrintfCsv(m_strLogFileName,
											"[HLS通信検査] SW%d, Adr%d 通信チェックエラー\n", 
											jj+1, pTestPat[ii].nComStartAdr[jj]);
							}
							else{
								LogPrintfCsv(m_strLogFileName,
											"[HLS通信検査] SW%d, Adr%d 非通信チェックエラー\n", 
											jj+1, pTestPat[ii].nComStartAdr[jj]);
							}
							printf("何かキーを押してください。\a\n");
							HitAnyKeyWait();
							bComStat = FALSE;
						}
					}
				}
				if( bComStat != TRUE ){
					bRetCont = FALSE;
					break;
				}
				else{
					//経過表示
					if((DWORD)(GetTickCount()-dwStartTick) > (DWORD)(pTestPat[ii].dwChkDurMsec-((pTestPat[ii].dwChkDurMsec/4) * 1))){
						printf("\r････\r");
					}
					else if((DWORD)(GetTickCount()-dwStartTick) > (DWORD)(pTestPat[ii].dwChkDurMsec-((pTestPat[ii].dwChkDurMsec/4) * 2))){
						printf("\r･･･\r");
					}
					else if((DWORD)(GetTickCount()-dwStartTick) > (DWORD)(pTestPat[ii].dwChkDurMsec-((pTestPat[ii].dwChkDurMsec/4) * 3))){
						printf("\r･･\r");
					}
					else if((DWORD)(GetTickCount()-dwStartTick) > 0){
						printf("\r･\r");
					}
				}
			}while((GetTickCount()-dwStartTick) < pTestPat[ii].dwChkDurMsec);
		}
		if( bRetCont != TRUE )break;
	}

	printf("\n");

	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"HLS通信チェック：%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:HLS通信チェック(NET%d) %s %s FINISH ■■■\n", nInsID, nComID+1, strDate, strTime);
	
	return bRetCont;
}

///////////////////////////////////////////////////////
//DIO折り返し検査
BOOL	SPxxxxDioChkAuto(int nInsID, BOOL bManualTest, BOOL bAutoTest)
{
	BOOL	bRetCont=TRUE;
	BOOL	berr = TRUE;
	int		nInputVal = -1;
	char	strDate[32];
	char	strTime[32];

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	lstrcpy(m_strLogFileName, "SPxxxx_DioChk.log");

	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s DIO検査(AUTO) %s %s START ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);

	//検査基板がSP_UPPER1 or 2のとき
	if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
		if( bManualTest == TRUE ){
			if( m_nIniSpUpperBoardType == 1 ){	//Upper2?
				berr = SPxxxxDioChkManualUpper2();	//Upper2 I/O検査（手動）
			}
			else{
				berr = SPxxxxDioChkManualUpper1();	//Upper1 I/O検査（手動）
			}
			if( berr != TRUE ){
				bRetCont = berr;
				printf("このまま検査を継続しますか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					berr = TRUE;	//次検査に移行する
				}
			}
		}
		if( berr == TRUE && bAutoTest == TRUE){
			if( m_nIniSpUpperBoardType == 1 ){	//Upper2
				bRetCont = SPxxxxDioChkAutoUpper2();	//Upper2 I/O検査（自動）
			}
			else{
				bRetCont = SPxxxxDioChkAutoUpper1();	//Upper1 I/O検査（自動）
			}
		}
	}
	//検査基板がSP_FRONT2のとき
	else if( m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ){
		bRetCont = SPxxxxDioChkAutoFront2();
	}
	//検査基板がSPINまたはSP_MOTHERのとき
	else if( SP_BID_MOTHER_BOARD ){
		if( m_nIniSpMotherBoardType != 0 ){	//SPIN_MOTHER_BOARD
			bRetCont = SPxxxxDioChkAutoSpinMother();
		}
		else{	//SP_MOTHER_BOARD
		}
	}

	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"DIO検査(AUTO)：%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	/////////////////////////////////////////////////////////////////////
	//DIO検査終了処理
	RS_SpNormalMode();
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s DIO検査(AUTO) %s %s FINISH ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

////////////////////////////////////////////
//SPIN_MOTHER_BOARD SAVENETを利用したDIO検査
BOOL	SPxxxxDioChkAutoSpinMother()
{
	BOOL	bRetCont=TRUE;
	BOOL	bZeroChkEnable;
	//BOOL	bHlsStopTest = TRUE;
	int		nInputVal;
	int		ii;

	WORD	wAnalogBitMask;
	//WORD	wTestPatten[16];
	int		nPatternCnt=0;

/***
	/////////////////////////////////////////////////////////////////////
	//HLS通信停止時の出力OFFテスト
	/////////////////////////////////////////////////////////////////////
	printf("\n");
	printf("-----------------------------------------------\n");
	printf(" HLS通信停止時の出力OFF確認\n");
	printf("-----------------------------------------------\n");
	memset(wTestPatten, 0, sizeof(wTestPatten));
	//CN61 OUT1_0～OUT1_f 出力OFFテスト
	wTestPatten[nPatternCnt++] = TEST_PATTERN10;
	//CN33 OUT2_0～OUT2_3 出力OFFテスト
	wTestPatten[nPatternCnt++] = TEST_PATTERN15;
	//CN41 OUT3_0～OUT3_b 出力OFFテスト
	wTestPatten[nPatternCnt++] = TEST_PATTERN16;
	//CN42 OUT3_0～OUT3_b 出力OFFテスト
	wTestPatten[nPatternCnt++] = TEST_PATTERN18;

	while(1){
		if( HlsOutStopTest(m_wSpinTestPattern, wTestPatten, nPatternCnt, 300) != TRUE ){
			printf("\n検査を中止しますか？(y or n) -->");
			nInputVal = KeyInputWaitYorN();
			if( nInputVal != 'y' ){
				continue;
			}
			bRetCont = FALSE;
		}
		break;
	}
**/
	/////////////////////////////////////////////////////////////////////
	//DI検査 (1点単位の入力検査)
	/////////////////////////////////////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n\n");
		printf("-----------------------------------------------\n");
		printf("SPIN_MOTHER_BOARD: DIO検査を開始しました。\n");
		for( ii=TEST_PATTERN0; ii<TEST_PATTERN0+SPIN_TEST_PATTERN_NUM; ii++ ){
			/*
			if( ii == TEST_PATTERN2 || ii == TEST_PATTERN8 || ii == TEST_PATTERN9 || ii == TEST_PATTERN15 || ii == TEST_PATTERN16){
				bZeroChkEnable = FALSE;	//パターン2, 8, 9, 15, 16では複数ON入力があるため他AdrのZeroチェックはしない
			}
			else{
				bZeroChkEnable = TRUE;
			}
			*/
			bZeroChkEnable = TRUE;		//とりあえず全てのテストパターンで他AdrのZeroチェックを実施しておく

			if( m_wSpinTestPattern[ii].wInAdr >= SP_XXXX_SW2_START_ADR+1 ){
				wAnalogBitMask = 0xc000;	//アナログ入力アドレスは下位14bitを0でマスクする
			}
			else{
				wAnalogBitMask = DIO_TEST_ALL_BIT;
			}
			
			if( bRetCont == TRUE ){
				bRetCont = DioAutoTestSelPattern(m_wSpinTestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);
				if( bRetCont != TRUE ){
					printf("\n検査を中止しますか？(y or n) -->");
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						break;
					}
				}
			}
			else{
				DioAutoTestSelPattern(m_wSpinTestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);	//NG発生時も引き続き検査を続行
				//break;
			}
		}
	}
	return bRetCont;
}

///////////////////////////////////////////////////////
//デジタル入出力複合テスト(SPIN_MOTHER_BOARD)
enum{
	S1_SW=1,
	S2_SW,
	S3_SW,
	S4_SW,
};
#define READCHK_LOOP_NUM		32		//読み出し繰り返し回数
BOOL	SPxxxxDioChkOtherSpinMother(int nInsID)
{
	BOOL	bRetCont=TRUE;
	BOOL	berr = TRUE;
	long	lnDiWaitVal[64];
	int		nInputVal = -1;
	DWORD	dwInData;
	int		ii;
	char	strDate[32];
	char	strTime[32];

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	lstrcpy(m_strLogFileName, "SPxxxx_DioChk.log");

	if( BoardId == 0xffff ) return FALSE;

	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s DIO検査(OTHER) %s %s START ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);


	//入力待ちデータエリアを無効値で初期化する
	for(ii=0;ii<64;ii++){
		lnDiWaitVal[ii] = -1;
	}
	///////////////////////////////////////////////////
	//SW操作による入力 (S4-4番がONのときの)確認
	printf("\n");
	printf("-------------------------------------------------\n");
	printf("   S4スイッチの4番のみをONに設定してください。\n");
	printf("   S1～S3スイッチは全てOFFに設定してください。\n");
	printf("-------------------------------------------------\n");
	printf("\n");

	//IN2-f, IN3-fがONになるのを待つ
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x8000;	//IN2
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x8000;	//IN3
	bRetCont = SnDiReadWait(lnDiWaitVal, 500);

	////////////////////////////
	//S1-1～8スイッチ入力確認
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S1_SW, 1, (WORD)(SP_XXXX_SW1_START_ADR+0), 0, 8);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S1-1～8スイッチ入力テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}
	////////////////////////////
	//S2-1～8スイッチ入力確認
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S2_SW, 1, (WORD)(SP_XXXX_SW1_START_ADR+1), 0, 8);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S2-1～8スイッチ入力テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}
	/////////////////////////////////////
	//OUT1_0～3 → IN3_7～a入力確認(ここは自動)
	if( bRetCont == TRUE ){
		printf("\n");
		printf("---------------------------------------------\n");
		printf("OUT1_0～3 → IN3_7～a 折り返し入力テスト中...\n");
		for( ii=0; ii<4; ii++ ){
			DATAout(BoardId, (WORD)(SP_XXXX_SW1_START_ADR+0), (WORD)(0x0001<<ii));

			//IN3-7～IN3-aが順にONになるのを待つ
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x8000;	//IN2
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x8000 | (0x0080 << ii);	//IN3
			bRetCont = SnDiReadWait(lnDiWaitVal, 100);
			if( bRetCont == TRUE ){
				//IN3-7～IN3-aがOFFになるのを待つ
				lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
				lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x8000;	//IN2
				lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x8000;	//IN3
				bRetCont = SnDiReadWait(lnDiWaitVal, 100);
			}		
			if( bRetCont != TRUE ){
				break;
			}
		}
		LogPrintfCsv(m_strLogFileName, "OUT1_0～3 → IN3_7～a 折り返し入力テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}	

	////////////////////////////
	//S3-1～4スイッチ入力確認
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S3_SW, 1, (WORD)(SP_XXXX_SW1_START_ADR+2), 0xb, 4);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S3-1～4スイッチ入力テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	////////////////////////////
	//S4-3スイッチ入力確認
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S4_SW, 3, (WORD)(SP_XXXX_SW1_START_ADR+1), 0xe, 1);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S4-3スイッチ入力テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	////////////////////////////
	//S4-4番をOFFに戻す
	printf("\n");
	printf("-------------------------------------------------\n");
	printf("   S4スイッチの4番をOFFに戻してください。\n");
	printf("-------------------------------------------------\n");
	printf("\n");

	//IN2-f, IN3-fがOFFになるのを待つ
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x0000;	//IN2
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x0000;	//IN3
	bRetCont = SnDiReadWait(lnDiWaitVal, 500);

	///////////////////////////////////////////////////
	//SW操作による入力 (S4-4番がOFFのときの)確認
	////////////////////////////
	//S4-1～2スイッチ入力確認
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S4_SW, 1, (WORD)(SP_XXXX_SW1_START_ADR+1), 0xe, 2);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S4-1～2スイッチ入力テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	//CN14(#51-B4:OFF(24V), #51-B5:ON(G))→IN3_e 入力ON確認(Auto)
	if( bRetCont == TRUE ){
		printf("\n");
		printf("---------------------------------------------\n");
		printf("CN14(SIG_24V,GND)入力 → IN3_4折り返し入力テスト中...\n");

		DATAout(BoardId, ADR_DO_51, 0x0030);	//(#51) B4:ON(SIG-24V), B5:ON(SIG-GND)
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x0000;	//IN2
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x0010;	//IN3 (B4:ON)
		bRetCont = SnDiReadWait(lnDiWaitVal, 100);
		//以下は条件が揃わないときにIN3-B4の入力がOFFになることの確認
		if( bRetCont == TRUE ){
			DATAout(BoardId, ADR_DO_51, 0x0020);	//(#51) B4:OFF, B5:ON
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x0000;	//IN3 (B4:OFF)
			bRetCont = SnDiReadWait(lnDiWaitVal, 100);
		}
		if( bRetCont == TRUE ){
			DATAout(BoardId, ADR_DO_51, 0x0010);	//(#51) B4:ON, B5:OFF
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x0000;	//IN3 (B4:OFF)
			bRetCont = SnDiReadWait(lnDiWaitVal, 100);
		}
		if( bRetCont == TRUE ){
			DATAout(BoardId, ADR_DO_51, 0x0000);	//(#51) B4:OFF, B5:OFF
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x0000;	//IN3 (B4:OFF)
			bRetCont = SnDiReadWait(lnDiWaitVal, 100);
		}

		LogPrintfCsv(m_strLogFileName, "CN14(SIG_24,GND)入力 → IN3_4折り返し入力テスト%s\n",
					((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	//以下複合パターンの検査(nMotor_Stopなど･･･)
	///////////////////////////////////////
	//nFFU Stop(S1-1)テスト
	//CN12(19_FFU_Stop_a):#53-B0 → CN12(20_FFU_Stop_b):#58-B0 が折り返すことを確認する
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ＜nFFU_Stopテスト＞\n");
		printf("   S1スイッチの1番のみをONに設定してください。\n");
		printf("------------------------------------------------------\n");
		bRetCont = DioLoopBackTest(ADR_DO_53, 0, ADR_DI_58, 0);
		LogPrintfCsv(m_strLogFileName, "(S1-1):nFFU_Stopテスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("------------------------------------------------------\n");
	}

	///////////////////////////////////////
	//nCamber_SV_Stop(S1-2)テスト
	//LED:[B-CN23]･･･5(←CN22-2), 6(←CN21-1), 7(←CN21-2)の点灯/消灯を目視確認する
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ＜nCamber_SV_Stop＞\n");
		printf("   S1スイッチの2番のみをONにすることで、\n");
		printf("   検査機前面パネル[B-CN23]のLED5～LED7がONすることを確認してください。\n");
		printf("------------------------------------------------------\n");
		printf("   [B-CN23]LED5～LED7は正常に点灯していますか？(y or n) -->");
		nInputVal = KeyInputWaitYorN();
		if( nInputVal == 0x1B ){
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED5～LED7目視確認を中断しました\n");
			bRetCont = FALSE;		//検査中断
		}
		else if( nInputVal == 'y' ){
			//LED 点灯・消灯目視検査OK
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED5～LED7目視確認OK\n");
		}
		else{
			//LED 点灯・消灯目視検査NG
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED5～LED7目視確認NG\n");
			bRetCont = FALSE;		//電圧値異常
		}
	}

	///////////////////////////////////////
	//nMotor_Stop(S1-3)テスト
	//CN13 #52-B9 → IN5-A9
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ＜nFFU_Stopテスト＞\n");
		printf("   S1スイッチの3番のみをONに設定してください。\n");
		printf("------------------------------------------------------\n");
		bRetCont = DioLoopBackTest(ADR_DO_52, 9, (WORD)(SP_XXXX_SW1_START_ADR+1), 9);
		LogPrintfCsv(m_strLogFileName, "(S1-3):nMotor_Stopテスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("------------------------------------------------------\n");
	}

	///////////////////////////////////////
	// 24V_IL(S1-4)、(S1-6)テスト
	// S1-4,S1-6 ON → [B-CN23] 1～4 LED目視確認
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ＜24V_IL＞\n");
		printf("   S1スイッチの4番と6番のみをONにすることで、\n");
		printf("   検査機前面パネル[B-CN23]のLED1～LED4がONすることを確認してください。\n");
		printf("------------------------------------------------------\n");
		printf("   [B-CN23]LED1～LED4は正常に点灯していますか？(y or n) -->");
		nInputVal = KeyInputWaitYorN();
		if( nInputVal == 0x1B ){
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED1～LED4目視確認を中断しました\n");
			bRetCont = FALSE;		//検査中断
		}
		else if( nInputVal == 'y' ){
			//LED 点灯・消灯目視検査OK
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED1～LED4目視確認OK\n");
		}
		else{
			//LED 点灯・消灯目視検査NG
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED1～LED4目視確認NG\n");
			bRetCont = FALSE;		//電圧値異常
		}
		printf("------------------------------------------------------\n");
	}
	
	///////////////////////////////////////
	//nTower_SV_Stop(S1-5)テスト
	// #53-B2 → #58-A2、#51-B7 → #3-bit6
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ＜nTower_SV_Stopテスト＞\n");
		printf("   S1スイッチの5番のみをONに設定してください。\n");
		printf("------------------------------------------------------\n");

		bRetCont = DioLoopBackTest(ADR_DO_53, 2, ADR_DI_58, 2);
		LogPrintfCsv(m_strLogFileName, "(S1-5):Tower IL bテスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));

		berr = DioLoopBackTest(ADR_DO_51, 7, (WORD)(SP_XXXX_SW1_START_ADR+1), 6);
		LogPrintfCsv(m_strLogFileName, "(S1-5):Tower IL INテスト%s\n", ((berr == TRUE) ? "OK" : "NG"));
		if( berr != TRUE ){
			bRetCont = FALSE;
		}

		printf("------------------------------------------------------\n");
	}

	///////////////////////////////////////
	//IPA_Dispense_IL(S2-1)テスト
	// S2-1 → #56-A2
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x8000);	//OUT2-F ON
		bRetCont = SpMotherSxSwTest(S2_SW, 1, ADR_DI_56, 2, 1);
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x0000);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S2-1スイッチ入力テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}
	
	///////////////////////////////////////
	//Reserve_Dispense_IL(S2-2)テスト
	// S2-2 → #56-A1
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x4000);	//OUT2-E ON
		bRetCont = SpMotherSxSwTest(S2_SW, 2, ADR_DI_56, 1, 1);
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x0000);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S2-2スイッチ入力テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	///////////////////////////////////////
	//H2O4_Dispense_IL(S2-3)テスト
	// S2-3 → #56-A0
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x2000);	//OUT2-D ON
		bRetCont = SpMotherSxSwTest(S2_SW, 3, ADR_DI_56, 0, 1);
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x0000);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S2-3スイッチ入力テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	//////////////////////////////////////
	// CN13-9,CN13-11,CN31-15 → CN31-1,CN33-B1 AND回路テスト
	if( bRetCont == TRUE ){
		//
		//#52-B8(IL_IG_SW_a) ,#52-B9(EMG_1a) →#58-A3(EMG) 入力テスト
		//
	
		//条件が揃わないときに#58-A3,A4の入力がOFFになることの確認
		if( bRetCont == TRUE ){
			DATAout(BoardId, ADR_DO_52, 0x0100);	//(#51) B4:OFF, B5:ON
			Sleep_Cnt(100);
			for( ii=0; ii<READCHK_LOOP_NUM; ii++ ){
				Sleep_Cnt(5);
				DATAin(BoardId, ADR_DI_58, (LPINT*)&dwInData);
				if( (dwInData & 0x0018) != 0 ){
					bRetCont = FALSE;
					break;
				}
			}
		}
		if( bRetCont == TRUE ){
			DATAout(BoardId, ADR_DO_52, 0x0200);	//(#51) B4:ON, B5:OFF
			Sleep_Cnt(100);
			for( ii=0; ii<READCHK_LOOP_NUM; ii++ ){
				Sleep_Cnt(5);
				DATAin(BoardId, ADR_DI_58, (LPINT*)&dwInData);
				if( (dwInData & 0x0018) != 0 ){
					bRetCont = FALSE;
					break;
				}
			}
		}
		if( bRetCont == TRUE ){
			DATAout(BoardId, ADR_DO_52, 0x0000);	//(#51) B4:OFF, B5:OFF
			Sleep_Cnt(100);
			for( ii=0; ii<READCHK_LOOP_NUM; ii++ ){
				Sleep_Cnt(5);
				DATAin(BoardId, ADR_DI_58, (LPINT*)&dwInData);
				if( (dwInData & 0x0018) != 0 ){
					bRetCont = FALSE;
					break;
				}
			}
		}

		//条件が揃ったときに#58-A3の入力がONになることの確認
		if( bRetCont == TRUE ){
			DATAout(BoardId, ADR_DO_52, 0x0300);	//#52 B8,B9 ON
			Sleep_Cnt(100);
			for( ii=0; ii<READCHK_LOOP_NUM; ii++ ){
				Sleep_Cnt(5);
				DATAin(BoardId, ADR_DI_58, (LPINT*)&dwInData);
				if( (dwInData & 0x0018) != 0x0008 ){
					bRetCont = FALSE;
					break;
				}
			}
		}

		//#58-A3がONすることを確認する
		if( bRetCont == TRUE ){
			DATAout(BoardId, ADR_DO_52, 0x0302);	//#52 B1,B8,B9 ON
			Sleep_Cnt(100);
			for( ii=0; ii<READCHK_LOOP_NUM; ii++ ){
				Sleep_Cnt(5);
				DATAin(BoardId, ADR_DI_58, (LPINT*)&dwInData);
				if( (dwInData & 0x0018) != 0x0018 ){
					bRetCont = FALSE;
					break;
				}
			}
		}
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "CN13-9,CN13-11,CN31-15 → CN31-1,CN33-B1 AND回路テスト%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}
	
	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"DIO検査(OTHER)：%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	/////////////////////////////////////////////////////////////////////
	//DIO検査終了処理
	RS_SpNormalMode();
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s DIO検査(OTHER) %s %s FINISH ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}
//////////////////////////////////////////////////////
//  SP_MOTHER, SPIN_MOTHER用 S1～S4スイッチ入力検査
BOOL SpMotherSxSwTest(int nSwNo, int nSwStart, WORD wInAdr, WORD wInStartBit, int nBitCount)
{
	long	lnDiWaitVal[64];
	BOOL	bRet=TRUE;
	int		ii;

	//入力待ちデータエリアを無効値で初期化する
	for(ii=0;ii<64;ii++){
		lnDiWaitVal[ii] = -1;
	}

	for( ii=0; ii<nBitCount; ii++ ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   S%dスイッチの%d番のみをONに設定してください。\n", nSwNo, nSwStart+ii);
		printf("------------------------------------------------------\n");
		printf("\n");

		//IN1～IN3の該当bitのみがONになるのを待つ
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x8000;	//IN2 (bit15は常時ON)
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x8000;	//IN3 (bit15は常時ON)
		lnDiWaitVal[wInAdr] = lnDiWaitVal[wInAdr] | (0x0001 << (wInStartBit+ii));
		bRet = SnDiReadWait(lnDiWaitVal, 500);
		if( bRet == TRUE ){
			printf("\n");
			printf("------------------------------------------------------\n");
			printf("   S%dスイッチの%d番をOFFに戻してください。\n", nSwNo, nSwStart+ii);
			printf("------------------------------------------------------\n");
			printf("\n");

			//IN1～IN3がOFFになるのを待つ
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x8000;	//IN2
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x8000;	//IN3
			bRet = SnDiReadWait(lnDiWaitVal, 500);
		}
		if( bRet != TRUE ){
			break;
		}
	}
	return bRet;
}

//////////////////////////////////////////////////
//入力値が安定して読み出せるようになるまで待つ
#define	DI_READ_CHK_COUNT			(30)
BOOL	SnDiReadWait(long *plnDiWaitVal, DWORD dwFirstWaitMsec)	//[64]のうち配列[0]は使わない
{
	BOOL	bRet=TRUE;
	int		bInputChk;
	int		nInputVal;
	int		nTestCountNum=0;
	DWORD	dwInData;
	WORD	wChkAdr;

	if( BoardId == 0xffff ){
		return FALSE;
	}
	printf("\n入力値確認中｡｡｡[1/%d]\n", DI_READ_CHK_COUNT);

	while( bRet == TRUE ){
		if( _kbhit() ){
			nInputVal = _getch();
			if( nInputVal == 0x1B ){
				bRet = FALSE;
				LogPrintfCsv(m_strLogFileName, "[NG] 入力待ちをキャンセルしました\n");
				printf("\a");
				break;
			}
		}

		bInputChk = TRUE;
		for(wChkAdr=1; wChkAdr<=63; wChkAdr++){
			//入力待ちデータは有効なデータか？
			if( plnDiWaitVal[wChkAdr] >= 0x0000 && plnDiWaitVal[wChkAdr] <= 0xffff ){
				DATAin(BoardId, wChkAdr, (LPINT*)&dwInData);
				//////////////////////////////////////////////////
				//アナログ入力アドレスならば、下位12bitを0でマスクする
				switch(m_nSPxxxxBoardID){
				case SP_BID_UPPER_IF_BOARD:
					if( wChkAdr >= SP_XXXX_SW1_START_ADR+3 && wChkAdr <= SP_XXXX_SW3_START_ADR+0 ){
						dwInData = dwInData & 0xc000;
					}
					break;
				case SP_BID_FRONT_IF_BOARD:
					if( wChkAdr >= SP_XXXX_SW1_START_ADR+3 && wChkAdr <= SP_XXXX_SW2_START_ADR+2 ){
						dwInData = dwInData & 0xc000;
					}
					break;
				case SP_BID_MOTHER_BOARD:
					if( m_nIniSpMotherBoardType == 0 ){
						//SP_MOTHER_BOARD
						if( wChkAdr == SP_XXXX_SW2_START_ADR+2 ){
							dwInData = dwInData & 0xc000;
						}
					}
					else{
						//SPIN_MOTHER_BOARD
						if( wChkAdr == SP_XXXX_SW2_START_ADR+1 ){
							dwInData = dwInData & 0xc000;
						}
					}
					break;
				}

				//////////////////////////////////////////////////
				//入力データが一致するかを確認する
				if( (DWORD)plnDiWaitVal[wChkAdr] != dwInData ){
					//入力データ不一致
					bInputChk = FALSE;
					if( nTestCountNum > 0 ){
						printf("\r[NG] Adr=%d, WAIT=%04lX, DiVal=%04X [%d/%d] \n",
							wChkAdr, plnDiWaitVal[wChkAdr], dwInData,
							nTestCountNum, DI_READ_CHK_COUNT);
					}
					break;
				}
				else{
					//入力データOK
					printf("\r[OK] Adr=%d, WAIT=%04lX, DiVal=%04X [%d/%d] \r",
						wChkAdr, plnDiWaitVal[wChkAdr], dwInData,
						nTestCountNum, DI_READ_CHK_COUNT);
				}
			}
		}
		
		if( bRet == TRUE && bInputChk == TRUE ){
			if( nTestCountNum == 0 ){	//初回一致確認後は、入力チャタリング防止用にWAITを延ばす
				Sleep_Cnt(dwFirstWaitMsec);
			}
			nTestCountNum++;
			//DI_READ_CHK_COUNT回連続してデータが読み出せればOKとする
			if( nTestCountNum >= DI_READ_CHK_COUNT ){
				break;		//入力OK
			}
			printf("\n入力値確認中｡｡｡[%d/%d]\n", nTestCountNum+1, DI_READ_CHK_COUNT);
		}
		else{
			if( nTestCountNum > 0 ){
				LogPrintfCsv(m_strLogFileName, "\n[入力値が不安定です] Adr=%d, WAIT=%04lX, DiVal=%04X\n",
							wChkAdr, plnDiWaitVal[wChkAdr], dwInData);
				nTestCountNum = 0;
				bRet = FALSE;
				break;
			}
		}
		Sleep_Cnt(10);
	}	//while end (入力待ち)

	return bRet;
}

///////////////////////////////////////////////////////
//A/D入力精度確認(SP_XXXX_BOARD用)
#define	AD_INPUT_RANGE_4_20mA		(0)
#define	AD_INPUT_RANGE_0_5V			(1)
#define	AD_INPUT_RANGE_0_10V		(2)
#define	AD_INPUT_RANGE_NUM			(AD_INPUT_RANGE_0_10V+1)
#define	AD_INPUT_RANGE_CHK_NUM		(5)
double	m_fAdInputStdVal[AD_INPUT_RANGE_NUM][AD_INPUT_RANGE_CHK_NUM]={
	//MIN,  MID,  MAX, OVR(L), OVR{H)
	{ 4.0, 12.0, 20.0,   3.96,  20.04},		//4-20mAレンジ時の確認用の値
	{ 0.0,  2.5,  5.0, -0.013,  5.013},		//0-5Vレンジ時の確認用の値
	{ 0.0,  5.0, 10.0, -0.025, 10.025}		//0-10Vレンジ時の確認用の値
};

// nInsID  : 検査ID番号
// nSch    : A/D変換開始ch(-1指定時はDefaultの0とする)
// nEch    : A/D変換終了ch(-1指定時はSP基板毎に割り当てられる最大ch番号とする)
BOOL SPxxxxAdinChk(int nInsID, int nSch, int nEch)
{
	BOOL	bRetCont=TRUE;
	BOOL	bKensaNg=TRUE;
	int		ii, jj;
	int		nInputVal = -1;
	int		nChNum;
	int		nAdChMin=0;
	int		nAdChMax=0;
	int		nAdAdrOffs=0;
	int		nAdMode=AD_INPUT_RANGE_4_20mA;
	int		nExitFlg = 0;
	int		nIinStdVal = 0;			//AD入力基準値（14bit(0x3ff=20mA)）
	char	strStdVal[16];
	DWORD	ADInputVal[SP_XXXX_ADCH_MAX][AD_INPUT_DATA_NUM];
	DWORD	ADDatAve[SP_XXXX_ADCH_MAX][AD_INPUT_RANGE_CHK_NUM];	//[ch1～4][4,12,20mA,OVR(L),OVR(H)]
	DWORD	ADDatMin[SP_XXXX_ADCH_MAX][AD_INPUT_RANGE_CHK_NUM];	//[ch1～4][4,12,20mA,OVR(L),OVR(H)]
	DWORD	ADDatMax[SP_XXXX_ADCH_MAX][AD_INPUT_RANGE_CHK_NUM];	//[ch1～4][4,12,20mA,OVR(L),OVR(H)]
	double	fwkmin[SP_XXXX_ADCH_MAX][AD_INPUT_RANGE_CHK_NUM];
	double	fwkmax[SP_XXXX_ADCH_MAX][AD_INPUT_RANGE_CHK_NUM];
	double	fwkave[SP_XXXX_ADCH_MAX][AD_INPUT_RANGE_CHK_NUM];
	WORD	wTermAdr;
	DWORD	dwInData;
	//DWORD	err;
	int		nAioUnitOutVal[AD_INPUT_RANGE_CHK_NUM];

	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	lstrcpy(m_strLogFileName, "SPxxxx_AdChk.log");

	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s A/D入力精度確認 %s %s START ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);

	//検査機内部接続をアナログ電圧入力検査モードに設定する
	SnMultiMeterMeasureSel(0, ANA_INOUT_SEL_IN, ANA_VI_SEL_VOLT, ANA_LOAD_SEL_OPEN);

	memset( strStdVal, 0, sizeof(strStdVal) );
	memset( ADInputVal, 0, sizeof(ADInputVal) );

	for( nChNum=0; nChNum<SP_XXXX_ADCH_MAX; nChNum++ ){
		for( jj=0; jj<AD_INPUT_RANGE_CHK_NUM; jj++ ){
			ADDatAve[nChNum][jj] = 0;
			ADDatMax[nChNum][jj] = 0;
			ADDatMin[nChNum][jj] = 0xffff;
			fwkmin[nChNum][jj] = 0.0;
			fwkmax[nChNum][jj] = 0.0;
			fwkave[nChNum][jj] = 0.0;
		}
	}

	switch(m_nSPxxxxBoardID){
	case SP_BID_UPPER_IF_BOARD:	//ch0～ch2:4-20mA, ch3～ch5:5V
		nAdChMax = 6;
		break;
	case SP_BID_FRONT_IF_BOARD:	//ch0～ch3:4-20mA
		nAdChMax = 4;
		break;
	case SP_BID_MOTHER_BOARD:	//ch0:10V
		nAdChMax = 1;
		break;
	}
	if( nSch >= 0 ){
		nAdChMin = nSch;
	}
	if( nEch >= 0 ){
		nAdChMax = nEch + 1;
	}

	for( nChNum=nAdChMin; nChNum<nAdChMax; nChNum++ ){		//AD ch数分繰り返す

		printf("\n-----------------------------------------\n");
		switch(m_nSPxxxxBoardID){
		case SP_BID_UPPER_IF_BOARD:
			nAdAdrOffs = 7;
			if( nChNum <= 2 ){		//ch0～ch2:4-20mA
				nAdMode = AD_INPUT_RANGE_4_20mA;
				printf("[A/D ch%d]4-20mA入力の精度確認を開始しました。", nChNum);
			}
			else{					//ch3～ch5:5V
				nAdMode = AD_INPUT_RANGE_0_5V;
				printf("[A/D ch%d]0-5V入力の精度確認を開始しました。", nChNum);
			}
			break;
		case SP_BID_FRONT_IF_BOARD:	//ch0～ch3:4-20mA
			nAdAdrOffs = 7;
			nAdMode = AD_INPUT_RANGE_4_20mA;
			printf("[A/D ch%d]4-20mA入力の精度確認を開始しました。", nChNum);
			break;
		case SP_BID_MOTHER_BOARD:	//ch0:10V
			if( m_nIniSpMotherBoardType == 0 ){
				nAdAdrOffs = 10;
			}
			else{
				nAdAdrOffs = 9;
			}
			nAdMode = AD_INPUT_RANGE_0_10V;
			printf("[A/D ch%d]0-10V入力の精度確認を開始しました。", nChNum);
			break;
		}
		//CONTEC AIOユニットからの出力基準値をセットする
		switch(nAdMode){
		case AD_INPUT_RANGE_4_20mA:
			nAioUnitOutVal[0] = m_nIniAO_4mA;
			nAioUnitOutVal[1] = m_nIniAO_12mA;
			nAioUnitOutVal[2] = m_nIniAO_20mA;
			nAioUnitOutVal[3] = m_nIniAO_4mA_OVR_L;
			nAioUnitOutVal[4] = m_nIniAO_20mA_OVR_H;
			break;
		case AD_INPUT_RANGE_0_5V:
			nAioUnitOutVal[0] = m_nIniAO_0_0V;
			nAioUnitOutVal[1] = m_nIniAO_2_5V;
			nAioUnitOutVal[2] = m_nIniAO_5_0V;
			nAioUnitOutVal[3] = m_nIniAO_0_OVR_L;
			nAioUnitOutVal[4] = m_nIniAO_5_OVR_H;
			break;
		case AD_INPUT_RANGE_0_10V:
			nAioUnitOutVal[0] = m_nIniAO_0_0V;
			nAioUnitOutVal[1] = m_nIniAO_5_0V;
			nAioUnitOutVal[2] = m_nIniAO_10_0V;
			nAioUnitOutVal[3] = m_nIniAO_0_OVR_L;
			nAioUnitOutVal[4] = m_nIniAO_10_OVR_H;
			break;
		}

		printf("\n-----------------------------------------\n");

		//ii=最小値(4mA, 0V),中間値(12mA, 2.5V, 5V),最大値(20mA, 5V, 10V),
		//OVR_L(3.952mA,-0.015V,-0.03V),OVR_H(20.048mA, 5.015V, 10.03V)の入力精度確認を実施
		for( ii=0; ii<AD_INPUT_RANGE_CHK_NUM; ii++ ){	//5回(MIN,MID,MAX,OVRL,OVRH)ループ
			while(ForeverLoop){
				bKensaNg = TRUE;
				//精度確認用のAD(14bit)入力基準値を設定する
				switch(ii){
				case 0:
					nIinStdVal = 0x0000;	//最小値
					break;
				case 1:
					nIinStdVal = 0x2000;	//中間値
					break;
				case 2:
					nIinStdVal = 0x3FFF;	//最大値
					break;
				case 3:
					nIinStdVal = 0x0000;	//OVR(L)
					break;
				case 4:
					nIinStdVal = 0x3FFF;	//OVR(H)
					break;
				}
				
				printf("\n--------------------------------------------------\n");
				//printf("シグナルソース・マルチメータを検査品のch%dに接続し、\n", nChNum);

				switch(nAdMode){
				case AD_INPUT_RANGE_4_20mA:
					sprintf(strStdVal, "%4.2fmA", m_fAdInputStdVal[nAdMode][ii]);
					//printf("シグナルソースから%sを出力してください。\n", strStdVal);
					break;
				case AD_INPUT_RANGE_0_5V:
					sprintf(strStdVal, "%4.2fV", m_fAdInputStdVal[nAdMode][ii]);
					//printf("シグナルソースから%sを出力してください。\n", strStdVal);
					break;
				case AD_INPUT_RANGE_0_10V:
					sprintf(strStdVal, "%4.2fV", m_fAdInputStdVal[nAdMode][ii]);
					//printf("シグナルソースから%sを出力してください。\n", strStdVal);
					break;
				}

				//printf("----------------------------------------------------\n");
				//printf("準備ができたら何かキーを押してください。\n\n");
				//nInputVal = KeyInputWait();
				//if ( nInputVal == 0x1B ){
				//	break;
				//}

				//CONTEC AIO-160802AY-USBのAO00から調整用の電圧を出力する
				AioUnitWriteAOVal(0, (long)nAioUnitOutVal[ii]);
				printf("AD ch%dの精度確認を開始しました。\n", nChNum);

				//AD入力サンプリング開始
				for( jj=0; jj<AD_INPUT_DATA_NUM; jj++ ){
					////RS-232C Mode 0で補正後のA/D入力値を取得する
					//ADInputVal[nChNum][jj] = (DWORD)RS_SpInputAdCorrectVal(nChNum);
					wTermAdr = (unsigned short)(nChNum+nAdAdrOffs);
					DATAin(BoardId, wTermAdr, (LPINT*)&dwInData);
					ADInputVal[nChNum][jj] = (DWORD)(dwInData & 0x3fff);

					//取得値を（改行なしで）画面に表示する
					if( ii <= 2 ){			//4,12,20mA 精度確認
						printf("\r[%04d/%04d] AD ch%d[%s]=0x%04lX\r", 
								jj+1, AD_INPUT_DATA_NUM, nChNum, strStdVal, ADInputVal[nChNum][jj]);
					}
					else if( ii == 3 ){		//OVR(L)確認
						printf("\r[%04d/%04d] AD ch%d[OVR(L)]=0x%04lX\r", 
								jj+1, AD_INPUT_DATA_NUM, nChNum, ADInputVal[nChNum][jj]);
					}
					else{					//OVR(H)確認
						printf("\r[%04d/%04d] AD ch%d[OVR(H)]=0x%04lX\r", 
								jj+1, AD_INPUT_DATA_NUM, nChNum, ADInputVal[nChNum][jj]);
					}
					Sleep_Cnt(10);
				}
				printf("\n");

				if( nExitFlg != 0 ){
					break;
				}
				else{
					//AD入力データの最大・最小・平均を取得する
					GetAdMinMaxAve(&ADInputVal[nChNum][0], &ADDatMin[nChNum][ii], &ADDatMax[nChNum][ii], &ADDatAve[nChNum][ii]);
					fwkmin[nChNum][ii] = (double)(fabs((double)(nIinStdVal - (int)ADDatMin[nChNum][ii])));
					fwkmin[nChNum][ii] = (double)(fwkmin[nChNum][ii] / (double)0x3fff) * 100.0;
					fwkave[nChNum][ii] = (double)(fabs((double)(nIinStdVal - (int)ADDatAve[nChNum][ii])));
					fwkave[nChNum][ii] = (double)(fwkave[nChNum][ii] / (double)0x3fff) * 100.0;
					fwkmax[nChNum][ii] = (double)(fabs((double)(nIinStdVal - (int)ADDatMax[nChNum][ii])));
					fwkmax[nChNum][ii] = (double)(fwkmax[nChNum][ii] / (double)0x3fff) * 100.0;

					//
					//精度確認の結果を表示する
					//
					//最小・中間・最大値の入力確認時は入力精度が±0.3%以内に入ることを確認する
					//(※)CONTEC AIOユニットは10V以上の出力はできないため、OVR_H検査は0x3fff貼り付きチェックでは無く範囲チェックを行うようにした
					if( (ii <= 2) || ((nAdMode==AD_INPUT_RANGE_0_10V) && (ii == 4))){
						printf("\n<ch%d %s ADin>\nMIN=0x%04X(精度 %f%% [%s])\nAVE=0x%04X(精度 %f%% [%s])\nMAX=0x%04X(精度 %f%% [%s])\n", 
								nChNum, strStdVal,
								ADDatMin[nChNum][ii], fwkmin[nChNum][ii], ((fwkmin[nChNum][ii] > AD_OK_RATE) ? "NG" : "OK"),
								ADDatAve[nChNum][ii], fwkave[nChNum][ii], ((fwkave[nChNum][ii] > AD_OK_RATE) ? "NG" : "OK"),
								ADDatMax[nChNum][ii], fwkmax[nChNum][ii], ((fwkmax[nChNum][ii] > AD_OK_RATE) ? "NG" : "OK") );

						if( fwkmin[nChNum][ii] > AD_OK_RATE || fwkave[nChNum][ii] > AD_OK_RATE || fwkmax[nChNum][ii] > AD_OK_RATE ){
							printf("ch%dのA/D入力精度が%4.2f%%を超えています。\a\n", nChNum, AD_OK_RATE);
							bKensaNg = FALSE;
						}

						LogPrintfCsv(m_strLogFileName, "----<(A/D ch%d) %s(0x%03X) 入力精度確認>---\n", nChNum, strStdVal, nIinStdVal);
						LogPrintfCsv(m_strLogFileName, "CheckType, AdVal, Precision(%%), Result\n");
						//MIN
						LogPrintfCsv(m_strLogFileName, "MIN, %03X, %f, %s\n",
							ADDatMin[nChNum][ii], fwkmin[nChNum][ii], ((fwkmin[nChNum][ii] > AD_OK_RATE) ? "NG" : "OK"));
						//MAX
						LogPrintfCsv(m_strLogFileName, "MAX, %03X, %f, %s\n",
							ADDatMax[nChNum][ii], fwkmax[nChNum][ii], ((fwkmax[nChNum][ii] > AD_OK_RATE) ? "NG" : "OK"));
						//AVE
						LogPrintfCsv(m_strLogFileName, "AVE, %03X, %f, %s\n",
							ADDatAve[nChNum][ii], fwkave[nChNum][ii], ((fwkave[nChNum][ii] > AD_OK_RATE) ? "NG" : "OK"));
					}
					//Over Range入力確認時は、入力値が下限(0x0000)、上限(0x3fff)に貼り付くことを確認する
					else{
						printf("\n<ch%d %s ADin>\nMIN=0x%04X[%s]\nAVE=0x%04X[%s]\nMAX=0x%04X[%s]\n", 
								nChNum, strStdVal,
								ADDatMin[nChNum][ii], ((ADDatMin[nChNum][ii] != (DWORD)nIinStdVal) ? "NG" : "OK"),
								ADDatAve[nChNum][ii], ((ADDatAve[nChNum][ii] != (DWORD)nIinStdVal) ? "NG" : "OK"),
								ADDatMax[nChNum][ii], ((ADDatMax[nChNum][ii] != (DWORD)nIinStdVal) ? "NG" : "OK") );
						if( (ADDatMin[nChNum][ii] != (DWORD)nIinStdVal) || (ADDatAve[nChNum][ii] != (DWORD)nIinStdVal) || (ADDatMax[nChNum][ii] != (DWORD)nIinStdVal) ){
							if( ii == 3 ){
								printf("ch%dのOver Range(L)値が0x%Xではありません。\a\n", nChNum, nIinStdVal);
							}
							else{
								printf("ch%dのOver Range(H)値が0x%Xではありません。\a\n", nChNum, nIinStdVal);
							}
							bKensaNg = FALSE;
						}

						if( ii == 3 ){
							LogPrintfCsv(m_strLogFileName, "----<(A/D ch%d) OVR(L)(0x%03X)>---\n", nChNum, nIinStdVal );
						}
						else{
							LogPrintfCsv(m_strLogFileName, "----<(A/D ch%d) OVR(H)(0x%03X)>---\n", nChNum, nIinStdVal );
						}
						LogPrintfCsv(m_strLogFileName, "CheckType, AdVal, Result\n");
						//MIN
						LogPrintfCsv(m_strLogFileName, "MIN, %03X, %s\n",
							ADDatMin[nChNum][ii], ((ADDatMin[nChNum][ii] != (DWORD)nIinStdVal) ? "NG" : "OK"));
						//MAX
						LogPrintfCsv(m_strLogFileName, "MAX, %03X, %s\n",
							ADDatMax[nChNum][ii], ((ADDatMax[nChNum][ii] != (DWORD)nIinStdVal) ? "NG" : "OK"));
						//AVE
						LogPrintfCsv(m_strLogFileName, "AVE, %03X, %s\n",
							ADDatAve[nChNum][ii], ((ADDatAve[nChNum][ii] != (DWORD)nIinStdVal) ? "NG" : "OK"));
					}

					if( bKensaNg == FALSE ){
						printf("再検査しますか？(y or n) -->\a\n");
						nInputVal = KeyInputWaitYorN();
						if( nInputVal == 'y' ){
							continue;			//再検査実施
						}
						else{
							bRetCont = FALSE;	//再検査未実施
						}
					}
				}

				break;
			}	//while end (検査NG時のリトライ用)
		}	//for end (MIN,MID,MAX,OVR(L),OVR(H)の5パターンでテスト)

		if( nExitFlg == 1 ) break;
	}	//for end (AD ch数分繰り返す↑)
	
	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"A/D入力精度確認：%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	/////////////////////////////////////////////////////////////////////
	//A/D入力精度確認終了処理
	//RS_SpNormalMode();
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s A/D入力精度確認 %s %s FINISH ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

///////////////////////////////////////////////////////
//A/D入力調整処理(SP_XXXX_BOARD用)
// nInsID  : 検査ID番号
// nSch    : A/D変換開始ch(-1指定時はDefaultの0とする)
// nEch    : A/D変換終了ch(-1指定時はSP基板毎に割り当てられる最大ch番号とする)
BOOL SPxxxxAdInAdj(int nInsID, int nSch, int nEch)
{
	BOOL	bRetCont=TRUE;
	int		nInputVal = -1;
	int		nAdVal=0;
	int		nAdMode=0;
	int		nChNum;
	int		nAdChMin=0;
	int		nAdChMax=0;
	int		nAdRawVal;
	int		nAdRawMinLimitVal=0;
	int		nAdRawMaxLimitVal=0;
	int		ii, jj;
	int		nExitFlg = 0;
	BOOL	bWarnFlg=FALSE;
	DWORD	ADInputVal[SP_XXXX_ADCH_MAX][AD_INPUT_DATA_NUM];
	DWORD	ADDatAve[SP_XXXX_ADCH_MAX][2];		//[ch0～n][4,20mA]
	DWORD	ADDatMin[SP_XXXX_ADCH_MAX][2];		//[ch0～n][4,20mA]
	DWORD	ADDatMax[SP_XXXX_ADCH_MAX][2];		//[ch0～n][4,20mA]
	DWORD	dwAdOffset[SP_XXXX_ADCH_MAX];
	DWORD	dwAdGain[SP_XXXX_ADCH_MAX];
	DWORD	dwAdOffsetTemp[SP_XXXX_ADCH_MAX];
	DWORD	dwAdGainTemp[SP_XXXX_ADCH_MAX];
	BOOL	ADDataSet = FALSE;	//4mA、20mAともサンプリング完了後にTRUEをセットする
	int		nAOvalWork=m_nIniAO_0_0V;

	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	lstrcpy(m_strLogFileName, "SPxxxx_AdChk.log");

	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s A/D入力調整 %s %s START ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);

	//検査機内部接続をアナログ電圧入力検査モードに設定する
	SnMultiMeterMeasureSel(0, ANA_INOUT_SEL_IN, ANA_VI_SEL_VOLT, ANA_LOAD_SEL_OPEN);
	
	memset( ADInputVal, 0, sizeof(ADInputVal) );

	for( ii=0; ii<SP_XXXX_ADCH_MAX; ii++ ){
		dwAdOffset[ii] = 0xffff;
		dwAdGain[ii] = 0xffff;
		dwAdOffsetTemp[ii] = 0xffff;
		dwAdGainTemp[ii] = 0xffff;
	}

	switch(m_nSPxxxxBoardID){
	case SP_BID_UPPER_IF_BOARD:	//ch0～ch2:4-20mA, ch3～ch5:5V
		nAdChMax = 6;
		break;
	case SP_BID_FRONT_IF_BOARD:	//ch0～ch3:4-20mA
		nAdChMax = 4;
		break;
	case SP_BID_MOTHER_BOARD:	//ch0:10V
		nAdChMax = 1;
		break;
	default:
		printf("\nBOARD識別IDが不明です\a\n");
		return FALSE;
	}
	if( nSch >= 0 ){
		nAdChMin = nSch;
	}
	if( nEch >= 0 ){
		nAdChMax = nEch + 1;
	}

	ADDataSet = FALSE;	//1chでも4mA、20mAともサンプリングが完了したらTRUEをセットする
	for( nChNum=nAdChMin; nChNum<nAdChMax; nChNum++ ){

		printf("\n-----------------------------------------\n");
		switch(m_nSPxxxxBoardID){
		case SP_BID_UPPER_IF_BOARD:
			if( nChNum <= 2 ){		//ch0～ch2:4-20mA
				nAdMode = AD_INPUT_RANGE_4_20mA;
				printf("[A/D ch%d]4-20mA入力の調整を開始しました。", nChNum);
			}
			else{					//ch3～ch5:5V
				nAdMode = AD_INPUT_RANGE_0_5V;
				printf("[A/D ch%d]0-5V入力の調整を開始しました。", nChNum);
			}
			break;
		case SP_BID_FRONT_IF_BOARD:	//ch0～ch3:4-20mA
			nAdMode = AD_INPUT_RANGE_4_20mA;
			printf("[A/D ch%d]4-20mA入力の調整を開始しました。", nChNum);
			break;
		case SP_BID_MOTHER_BOARD:	//ch0:10V
			nAdMode = AD_INPUT_RANGE_0_10V;
			printf("[A/D ch%d]0-10V入力の調整を開始しました。", nChNum);
			break;
		}
		printf("\n-----------------------------------------\n");

		//A/D調整を開始する
		for( ii=0; ii<2; ii++ ){	//ii: 0=4mA(or 0V), 1=20mA(or 5V,10V)
			while(ForeverLoop){
				///////////////////////////////////////////////////////
				//検査品のアナログ入力生データをｎ回サンプリングする
				ADDatAve[nChNum][ii] = 0;
				ADDatMax[nChNum][ii] = 0;
				ADDatMin[nChNum][ii] = 0xffff;

				printf("\n--------------------------------------------------\n");
				//printf("シグナルソース・マルチメータを検査品のch%dに接続し、\n", nChNum);

				switch(nAdMode){
				case AD_INPUT_RANGE_4_20mA:
					nAdVal = ((ii==0) ? 4 : 20);
					nAOvalWork = ((ii==0) ? m_nIniAO_4mA : m_nIniAO_20mA);
					nAdRawMinLimitVal = ((ii==0) ? INVALID_ADRAW_MINVAL_4mA : INVALID_ADRAW_MINVAL_20mA);
					nAdRawMaxLimitVal = ((ii==0) ? INVALID_ADRAW_MAXVAL_4mA : INVALID_ADRAW_MAXVAL_20mA);
					//printf("シグナルソースから%dmAを出力してください。\n", nAdVal);
					break;
				case AD_INPUT_RANGE_0_5V:
					nAdVal = ((ii==0) ? 0 : 5);
					nAOvalWork = ((ii==0) ? m_nIniAO_0_0V : m_nIniAO_5_0V);
					nAdRawMinLimitVal = ((ii==0) ? INVALID_ADRAW_MINVAL_F0V : INVALID_ADRAW_MINVAL_F5V);
					nAdRawMaxLimitVal = ((ii==0) ? INVALID_ADRAW_MAXVAL_F0V : INVALID_ADRAW_MAXVAL_F5V);
					//printf("シグナルソースから%dVを出力してください。\n", nAdVal);
					break;
				case AD_INPUT_RANGE_0_10V:
					nAdVal = ((ii==0) ? 0 : 10);
					nAOvalWork = ((ii==0) ? m_nIniAO_0_0V : m_nIniAO_10_0V);
					nAdRawMinLimitVal = ((ii==0) ? INVALID_ADRAW_MINVAL_T0V : INVALID_ADRAW_MINVAL_T10V);
					nAdRawMaxLimitVal = ((ii==0) ? INVALID_ADRAW_MAXVAL_T0V : INVALID_ADRAW_MAXVAL_T10V);
					//printf("シグナルソースから%dVを出力してください。\n", nAdVal);
					break;
				}

				//CONTEC AIO-160802AY-USBのAO00から調整用の電圧を出力する
				AioUnitWriteAOVal(0, (long)nAOvalWork);
				printf("AD ch%dの生データ入力を開始しました。\n", nChNum);

				for( jj=0; jj<AD_INPUT_DATA_NUM; jj++ ){
					nAdRawVal = RS_SpInputAdRawVal(nChNum);

#ifdef	_RS_DEBUG_MODE
nAdRawVal = nAdRawMinLimitVal+0x100;
#endif
					if( nAdRawVal < nAdRawMinLimitVal ){
						printf("<<WARNING>> ch%d AD RAW Value 0x%04X (値が低すぎる？)\n", nChNum, nAdRawVal);
						bWarnFlg = TRUE;
					}
					else if( nAdRawVal > nAdRawMaxLimitVal ){
						printf("<<WARNING>> ch%d AD RAW Value 0x%04X (値が高すぎる？)\n", nChNum, nAdRawVal);
						if( nAdRawVal == 0xffff ){
							printf("AD生値入力を中止します。よろしいですか？(y or n) -->");
							nInputVal = KeyInputWaitYorN();
							if( nInputVal == 'y' ){
								bRetCont = FALSE;
								nExitFlg = 1;
								break;
							}
						}
						bWarnFlg = TRUE;
					}
					else{	//改行せずに入力した値をそのまま表示する
						printf("\r[%04d/%04d] ch%d AD RAW Value=0x%04X\r",
								jj+1, AD_INPUT_DATA_NUM, nChNum, nAdRawVal);
					}
					ADInputVal[nChNum][jj] = (DWORD)nAdRawVal;
				}	//for jjの終わり（サンプリング完了）
				printf("\n");

				if( bWarnFlg == TRUE ){	//入力値異常？（警告あり）のとき
					bWarnFlg = FALSE;
					switch(nAdMode){
					case AD_INPUT_RANGE_4_20mA:
						printf("[(ch%d)%dmA] もう一度A/D生値入力を実施しますか？(y or n) -->", nChNum, nAdVal);
						break;
					case AD_INPUT_RANGE_0_5V:
					case AD_INPUT_RANGE_0_10V:
						printf("[(ch%d)%dV] もう一度A/D生値入力を実施しますか？(y or n) -->", nChNum, nAdVal);
						break;
					}
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						continue;
					}
					else{
						bRetCont = FALSE;
						nExitFlg = 1;
					}
				}
				break;
			}//Warning発生時の繰り返し用whileのend

			//AD入力生データの最大・最小・平均を取得する
			if( (nExitFlg == 0) && (bRetCont == TRUE) ){
				GetAdMinMaxAve(&ADInputVal[nChNum][0], &ADDatMin[nChNum][ii], &ADDatMax[nChNum][ii], &ADDatAve[nChNum][ii]);
				if( ii == 1 ){	//最大レンジ側の入力時
					//Offset/Gain計算値を表示する
					GetAdOffsetGain(nAdMode, ADDatAve[nChNum][0], ADDatAve[nChNum][1], &dwAdOffset[nChNum], &dwAdGain[nChNum]);

					LogPrintfCsv(m_strLogFileName, "(ch%d RawVAL) Lo[0x%lX] Hi[0x%lX] Offset[0x%lX] Gain[0x%lX]\n",
								nChNum, ADDatAve[nChNum][0], ADDatAve[nChNum][1], dwAdOffset[nChNum], dwAdGain[nChNum] );
					printf("\n");
					ADDataSet = TRUE;		//全ch 4,20mAとも生データのサンプリング完了
				}
			}
			if( nExitFlg == 1 ) break;

		}	//↑ii=入力最小レンジ側と最大レンジ側の２回取り込む
		
		if( nExitFlg == 1 ) break;
	}	//nChNum=AD ch数分ループ end

	/////////////////////////////////////////////////////////////////////
	//サンプリング値（平均値のみ使用）を元にして補正値を算出する
	if( (nExitFlg == 0) && (ADDataSet == TRUE) && (bRetCont == TRUE) ){
		printf("\n-----------------------------------------\n");
		printf("アナログ入力補正値更新中...\n");

		if( RS_SpSetAdOffsetGain(dwAdOffset, dwAdGain, NULL, NULL) == TRUE ){
			//CSVコメント行
			LogPrintfCsv(m_strLogFileName, "\n[AD_ch], [RawVAL(4mA)], [RawVAL(20mA)], [Offset], [Gain]\n");
			for( nChNum=nAdChMin; nChNum<nAdChMax; nChNum++ ){
				if( (dwAdOffset[nChNum] == 0xffff) || (dwAdGain[nChNum] == 0xffff) ){
					//ここには入らないはず
					printf("(ch%d) AD入力補正 <変更なし>。\n", nChNum);
				}
				else{
					LogPrintfCsv(m_strLogFileName, "%d, %04lX, %04lX, %03lX, %03lX\n", 
								nChNum, ADDatAve[nChNum][0], ADDatAve[nChNum][1], dwAdOffset[nChNum], dwAdGain[nChNum]);
				}
			}
		}
		else{
			LogPrintfCsv(m_strLogFileName, "アナログ入力補正値更新不可\n");
			bRetCont = FALSE;
		}
	}
	
	/////////////////////////////////////////////////////////////////////
	//A/D入力補正値のEEPROM書込み確認
	if( (nExitFlg == 0) && (ADDataSet == TRUE) && (bRetCont == TRUE) ){
		LogPrintfCsv(m_strLogFileName, "\n<<<EEPROM SAVE>>>\n");
		while(ForeverLoop){
			printf("\n-----------------------------------------\n");
			printf("EEPROM保存実行中...\n");
			bRetCont = TRUE;
			if( RS_SpUpdateEeprom(nAdChMax) == TRUE ){		//EEPROM保存実行
				printf("EEPROM保存が完了しました。\n\n");
				printf("\n-----------------------------------------\n");
				printf("EEPROMに保存されたデータを確認します。検査品の電源を再投入してください。\n");
				printf("準備ができたら何かキーを押してください。");
				KeyInputWait();
				printf("\n");
				if( RS_SpSetAdOffsetGain(dwAdOffsetTemp, dwAdGainTemp, dwAdOffsetTemp, dwAdGainTemp) == TRUE ){	//補正値の読出しのみ実行
					for( nChNum=nAdChMin; nChNum<nAdChMax; nChNum++ ){
						if( (dwAdOffset[nChNum] == 0xffff || dwAdGain[nChNum] == 0xffff) ||
							(dwAdOffsetTemp[nChNum] == dwAdOffset[nChNum] && dwAdGainTemp[nChNum] == dwAdGain[nChNum]) )
						{
							LogPrintfCsv(m_strLogFileName, " [OK]:(ch%d) Offset W[0x%04X] R[0x%04X], GAIN W[0x%04X] R[0x%04X]\n",
										nChNum, dwAdOffset[nChNum], dwAdOffsetTemp[nChNum],
										dwAdGain[nChNum], dwAdGainTemp[nChNum]);
						}
						else{
							//AD入力補正値が一致しません。
							bRetCont = FALSE;
							LogPrintfCsv(m_strLogFileName, " [NG]:(ch%d) Offset W[0x%04X] R[0x%04X], GAIN W[0x%04X] R[0x%04X]\n",
										nChNum, dwAdOffset[nChNum], dwAdOffsetTemp[nChNum],
										dwAdGain[nChNum], dwAdGainTemp[nChNum]);
						}
					}
				}
				else{
					printf("AD入力補正値が取得できません。\n");
					bRetCont = FALSE;
				}
				if( bRetCont != TRUE ){
					printf("AD入力補正値のEEPROMへの書き込みに失敗しました。再度、実行しますか？(y or n) -->");
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						continue;
					}
				}
			}
			else{
				printf("EEPROM保存に失敗しました。再度、実行しますか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					continue;
				}
				bRetCont = FALSE;
			}
			break;
		}
	}

	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"A/D入力調整終了処理：%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	/////////////////////////////////////////////////////////////////////
	//A/D入力調整終了処理
	RS_SpNormalMode();
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s A/D入力調整 %s %s FINISH ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

///////////////////////////////////////////////////////
//LED目視・消費電流確認
// nInsID  : 検査ID番号
BOOL SPxxxxLedTest(int nInsID)
{
	BOOL	bRetCont=TRUE;
	int		nInputVal=-1;
	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	lstrcpy(m_strLogFileName, "SPxxxx_VisualChk.log");

	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s LED目視確認／消費電流確認 %s %s START ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);

	///////////////////////////////////////
	//LEDの目視検査を実施する
	while( bRetCont == TRUE ){	//再検査用while
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("  ＜LED目視確認＞\n");
		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
			printf("   LD1～LD5の緑LEDが点灯していることを確認してください\n");
		}
		else if( m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ){
			printf("   LD1～LD7の緑LEDが点灯していることを確認してください\n");
		}
		else if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			if( m_nIniSpMotherBoardType == 0 ){		//SP_MOTHER_BOARD
				printf("   LD1～LD9の緑LEDが点灯していることを確認してください\n");
			}
			else{
				printf("   LD1～LD4、LD6～LD9の緑LEDが点灯していることを確認してください\n");
			}
		}
		printf("------------------------------------------------------\n");
		printf("\n");
		printf(" LEDはチラツキ・輝度不足なく正常に点灯していますか？(y or n) -->");
		nInputVal = KeyInputWaitYorN();
		if( nInputVal == 0x1B ){
			//検査中断
			LogPrintfCsv(m_strLogFileName, "[Cancel] LED点灯目視確認\n");
			bRetCont = FALSE;
			break;
		}
		else if( nInputVal == 'y' ){
			//検査OK
			LogPrintfCsv(m_strLogFileName, "[OK] LED点灯目視確認\n");
		}
		else{
			//検査NG
			LogPrintfCsv(m_strLogFileName, "[NG] LED点灯目視確認\n");
			bRetCont = FALSE;
		}

		if( bRetCont != TRUE ){
			printf("再検査しますか？(y or n) -->");
			nInputVal = KeyInputWaitYorN();
			if( nInputVal == 'y' ){
				bRetCont = TRUE;
				LogPrintfCsv(m_strLogFileName, "\n----再検査----\n");
				continue;			//再検査実施
			}
		}
		break;
	}	//while (再検査用)

	///////////////////////////////////////
	//消費電流の確認を実施する
	while( bRetCont == TRUE ){	//再検査用while
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("  ＜DC24V消費電流確認＞\n");
		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
				printf("   検査品の主電源(DC24V)の消費電流が250mA以下であることを確認してください\n");
		}
		else if( m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ){
				printf("   検査品の主電源(DC24V)の消費電流が250mA以下であることを確認してください\n");
		}
		else if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			if( m_nIniSpMotherBoardType == 0 ){		//SP_MOTHER_BOARD
				printf("   検査品の主電源(DC24V)の消費電流が250mA以下であることを確認してください\n");
			}
			else{	//SPIN_MOTHER_BOARD
				printf("   検査品の主電源(DC24V)の消費電流が280mA以下であることを確認してください\n");
			}
		}
		printf("------------------------------------------------------\n");
		printf("\n");
		printf(" 消費電流は正常ですか？(y or n) -->");
		nInputVal = KeyInputWaitYorN();
		if( nInputVal == 0x1B ){
			//検査中断
			LogPrintfCsv(m_strLogFileName, "[Cancel] 消費電流確認\n");
			bRetCont = FALSE;
			break;
		}
		else if( nInputVal == 'y' ){
			//検査OK
			LogPrintfCsv(m_strLogFileName, "[OK] 消費電流確認\n");
		}
		else{
			//検査NG
			LogPrintfCsv(m_strLogFileName, "[NG] 消費電流確認\n");
			bRetCont = FALSE;
		}

		if( bRetCont != TRUE ){
			printf("再検査しますか？(y or n) -->");
			nInputVal = KeyInputWaitYorN();
			if( nInputVal == 'y' ){
				bRetCont = TRUE;
				LogPrintfCsv(m_strLogFileName, "\n----再検査----\n");
				continue;			//再検査実施
			}
		}
		break;
	}	//while (再検査用)

	/////////////////////////////////////////////////////////////////////
	// 検査結果出力
	/////////////////////////////////////////////////////////////////////
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"LED目視確認・消費電流確認：%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("■■■■■■■■■■■■■■■■■■■■■■\n");

	/////////////////////////////////////////////////////////////////////
	//終了処理
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"■■■ %c:%s LED目視確認／消費電流確認 %s %s FINISH ■■■\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

//指定アドレス範囲の通信状態を確認する
//bCommChk: TRUE=通信中であることを確認、FALSE=非通信状態であることを確認
BOOL SpComChk(int nStartAdr, int nEndAdr, BOOL bCommChk)
{
	BOOL	berr = TRUE;
	DWORD	err;
	DWORD	termerr;            // ターミナル通信状態
	WORD	unit;

	for( unit=(WORD)nStartAdr; unit<=(WORD)nEndAdr; unit++ ){
		if( unit > 0 ){
			err = TERMchk(BoardId, unit, (LPINT*)&termerr);
			if(err != SNMA_NO_ERROR){
				printf("\nFailed to Get Terminal Communication Status!!\n");
				printf("何かキーを押してください。\a\n");
				HitAnyKeyWait();
				berr = FALSE;
				break;
			} 
			else{
				if(termerr & 0x7C00){
					if( bCommChk == TRUE ){
						//printf("Comm Error. UnitID=%2u. Com Status=0x%04X\n", unit, (WORD)termerr);
						berr = FALSE;
					}
				}
				else{
					if( bCommChk == FALSE ){
						berr = FALSE;
					}
				}
			}
		}
	}

	return berr;
}

///////////////////////////////////////////////////////
//D/A出力調整処理(SP_XXXX_BOARD用)
BOOL SPxxxxDaOutAdj()
{
	BOOL	bRetCont=TRUE;
	int		nInputVal = -1;
	int		nChNum;
	int		ii;
	//int		jj;
	//int		nExitFlg = 0;
	//BOOL	bWarnFlg=FALSE;
	DWORD	dwDaOffset[SP_XXXX_DACH_MAX];
	DWORD	dwDaGain[SP_XXXX_DACH_MAX];
	DWORD	dwDaOffsetTemp[SP_XXXX_DACH_MAX];
	DWORD	dwDaGainTemp[SP_XXXX_DACH_MAX];
	DWORD	dwTemp;
	DWORD	dwDaOffset_OldVal[SP_XXXX_DACH_MAX];
	DWORD	dwDaGain_OldVal[SP_XXXX_DACH_MAX];

	DWORD	dwDaOffset_GoodVal;
	DWORD	dwDaGain_GoodVal;
	double	fVoltage_GoodVal;

	//char	*endptr;
	//char	strwk[256];
	DWORD	err;
	WORD	wTermAdr;
	WORD	wDaOffset[4];
	WORD	wDaGain[4];
	double	fDaOffsetMultiMeter[4];
	double	fDaGainMultiMeter[4];
	//WORD	wDoVal = 0;
	//WORD	wDoValBak = 0;

	HANDLE	forMultiMaterHandles = NULL;
	BOOL	retStart_HP34401A=0;
	BOOL	retChangeVA_DC_HP34401A;
	double	fVoltageVal=0.0;

	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv("SPxxxx_DaLog.csv", 
		"■■■ [%s] D/A出力調整 %s %s START ■■■\n", m_cSPxxxxBoardName, strDate, strTime);

	//マルチメータで計測するための準備
	retStart_HP34401A = Start_HP34401A(
		m_cComPort,
		&forMultiMaterHandles	//通信リソースのハンドルを返すアドレス
	);
	if( ! retStart_HP34401A ){
		LogPrintf("MultiMeter 34401A Initialize Error!!\n");
		return FALSE;
	}

	//マルチメータのレンジを電圧入力に設定する
	retChangeVA_DC_HP34401A = ChangeVoltageDC_HP34401A(
		forMultiMaterHandles	//通信リソースハンドル
	);
	if( ! retChangeVA_DC_HP34401A ){
		LogPrintf("MultiMeter 34401A Voltage Setting Error!!\n");
		End_HP34401A(forMultiMaterHandles);	//通信リソースのハンドルを閉じる
		return FALSE;
	}

	for(ii=0; ii<SP_XXXX_DACH_MAX; ii++){
		dwDaOffset[ii] = 0xffff;
		dwDaGain[ii] = 0xffff;
		dwDaOffsetTemp[ii] = 0xffff;
		dwDaGainTemp[ii] = 0xffff;
		wDaOffset[ii] = 0xffff;
		wDaGain[ii] = 0xffff;
		fDaOffsetMultiMeter[ii] = 0.0;
		fDaGainMultiMeter[ii] = 0.0;
	}

	if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) == TRUE ){	//補正値（現在値）の読出しのみ実行
		memcpy(dwDaOffset_OldVal, dwDaOffset, sizeof(dwDaOffset_OldVal));
		memcpy(dwDaGain_OldVal, dwDaGain, sizeof(dwDaGain_OldVal));

		for( nChNum=0; nChNum<SP_XXXX_DACH_MAX; nChNum++ ){
			printf("\n---------------------------------------------------------\n");
			printf(" マルチメータのプローブをDA ch%d端子に接続してください。\n", nChNum);
			printf("-----------------------------------------------------------\n");
			printf("準備ができたら何かキーを押してください。\n\n");
			nInputVal = KeyInputWait();
			if ( nInputVal == 0x1B ){
				//return FALSE;
				bRetCont = FALSE;
				break;
			}

			while(ForeverLoop){	//調整リトライ用while
				//[モード3]アナログ出力補正モードに切り替えて、オフセット更新可能な状態とする
				printf("[D/A ch%d] Offset=0x%04X, Gain=0x%04X\n", nChNum, dwDaOffset[nChNum], dwDaGain[nChNum]);

				//printf("\n-----------------------------------------------------------------\n");
				//printf(" SNデモソフトからMKY(Adr%d)に0V(0x0000)出力後に調整を開始してください\n", nChNum+(SP_XXXX_SW2_START_ADR+2));
				//printf("------------------------------------------------------------------\n");

				wTermAdr = (unsigned short)(nChNum+(SP_XXXX_SW2_START_ADR+2));
				err = DATAout(BoardId, wTermAdr, (WORD)0x0000);
				if( err != SNMA_NO_ERROR ){
					printf("DATAout() err=%ld\n", err);
					printf("何かキーを押してください。\a\n");
					HitAnyKeyWait();
					return FALSE;
				}
				printf("\n-----------------------------------------------------------------\n");
				printf(" MKY(Adr%d)に0V(0x0000)出力を開始しました\n", wTermAdr);
				printf("------------------------------------------------------------------\n");

				//出力データをD11 →D0の順に１ビットずつONしていき、マルチメータからの入力値が
				//0V未満になればOFFに戻して、次のビットに移る
				if( bRetCont == TRUE ){
					printf("\nD/A(ch%d) 補正値(Offset)の調整中...\n\n", nChNum);
					dwDaOffset[nChNum] = 0x0000;
					dwDaGain[nChNum] = 0x0400;

					dwDaOffset_GoodVal = dwDaOffset[nChNum];
					dwDaGain_GoodVal = dwDaGain[nChNum];
					fVoltage_GoodVal = -999.9;

					for( ii=11; ii>=0; ii-- ){
						dwTemp = dwDaOffset[nChNum];
						dwDaOffset[nChNum] = dwTemp | (DWORD)(0x0001 << ii);
						//Offset補正値を更新する
						if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//変更後の値に更新する
							printf("[D/A ch%d] Offset補正値の更新に失敗しました。\a\a\n", nChNum);
							bRetCont = FALSE;
							break;
						}

						//マルチメータから電圧値を取得する
						Sleep_Cnt(1000);
						if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
							printf("電圧値を取得できませんでした。何かキーを押してください。\a\n");
							HitAnyKeyWait();
							bRetCont = FALSE;
							break;
						}
						printf("Offset=0x%04X, Gain=0x%04X [%fV]\n", dwDaOffset[nChNum], dwDaGain[nChNum], fVoltageVal);

						//目標値(0.0Vに最も近かったOFFSET/GAINを変数GoodValに記憶しておく)
						if( (double)fabs(fVoltageVal - 0.0) < (double)fabs(fVoltage_GoodVal - 0.0) ){
							dwDaOffset_GoodVal = dwDaOffset[nChNum];
							dwDaGain_GoodVal = dwDaGain[nChNum];
							fVoltage_GoodVal = fVoltageVal;
						}
						
						if( fVoltageVal > 1.0 ){
							printf("検査品のch%d からのD/A出力値が高すぎます。\a\n", nChNum);
							printf("出力値(0x0000), 調整基準値(0.0 V), マルチメータ値(%f V)\n", fVoltageVal);
							printf("hit any key.\a\n");
							HitAnyKeyWait();
							bRetCont = FALSE;
							break;
						}
						else if( fVoltageVal < 0.0 ){
							//入力値が0V未満ならば元に戻す（今回bitをOFFする）
							dwDaOffset[nChNum] = dwTemp;
						}

						if( ii == 0 ){	//最終bitのとき
							dwDaOffset[nChNum] = dwDaOffset_GoodVal;
							dwDaGain[nChNum] = dwDaGain_GoodVal;
							if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//変更後の値に更新する
								printf("[D/A ch%d] Offset補正値の更新に失敗しました。\a\a\n", nChNum);
								bRetCont = FALSE;
								break;
							}

							//マルチメータから最終の電圧値を取得する
							Sleep_Cnt(1000);
							if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
								printf("電圧値を取得できませんでした。何かキーを押してください。\a\n");
								bRetCont = FALSE;
								break;
							}
						}
					}
					//if( nExitFlg == 0 ){
						wDaOffset[nChNum] = (WORD)(dwDaOffset[nChNum] & 0x0fff);	//(WORD)(dwDaOffset & 0x0fff);
						printf("(ch%d) D/A Offset[0x%X] AdjOut[%fV]\n", nChNum, wDaOffset[nChNum], fVoltageVal);
						fDaOffsetMultiMeter[nChNum] = fVoltageVal;
					//}
				}
				
				//printf("\n-------------------------------------------------------\n");
				//printf(" SNデモソフトからMKY(Adr%d)に5V(0x3FFF)出力後に調整を開始してください\n", nChNum+(SP_XXXX_SW2_START_ADR+2));
				//printf("---------------------------------------------------------\n");
				err = DATAout(BoardId, wTermAdr, (WORD)0x3FFF);
				if( err != SNMA_NO_ERROR ){
					printf("DATAout() err=%ld\n", err);
					printf("何かキーを押してください。\a\n");
					HitAnyKeyWait();
					bRetCont = FALSE;
					break;
				}
				printf("\n-----------------------------------------------------------------\n");
				printf(" MKY(Adr%d)に5V(0x3FFF)出力を開始しました\n", wTermAdr);
				printf("------------------------------------------------------------------\n");

				//出力データをD11 →D0の順に１ビットずつONしていき、マルチメータからの入力値が
				//5Vを超えたらOFFに戻して、次のビットに移る
				if( bRetCont == TRUE ){
					printf("\nD/A(ch%d) 補正値(Gain)の調整中...\n\n", nChNum);
					dwDaGain[nChNum] = 0x0000;

					dwDaOffset_GoodVal = dwDaOffset[nChNum];
					dwDaGain_GoodVal = dwDaGain[nChNum];
					fVoltage_GoodVal = -999.9;

					for( ii=11; ii>=0; ii-- ){
						dwTemp = dwDaGain[nChNum];
						dwDaGain[nChNum] = dwTemp | (DWORD)(0x0001 << ii);
						//Gain補正値を更新する
						if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//変更後の値に更新する
							printf("[D/A ch%d] Offset補正値の更新に失敗しました。\a\a\n", nChNum);
							bRetCont = FALSE;
							break;
						}

						//マルチメータから電圧値を取得する
						Sleep_Cnt(1000);
						if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
							printf("電圧値を取得できませんでした。何かキーを押してください。\a\n");
							HitAnyKeyWait();
							bRetCont = FALSE;
							break;
						}

						printf("Offset=0x%04X, Gain=0x%04X [%fV]\n", dwDaOffset[nChNum], dwDaGain[nChNum], fVoltageVal);

						//目標値(5.0Vに最も近かったOFFSET/GAINを変数GoodValに記憶しておく)
						if( (double)fabs(fVoltageVal - 5.0) < (double)fabs(fVoltage_GoodVal - 5.0) ){
							dwDaOffset_GoodVal = dwDaOffset[nChNum];
							dwDaGain_GoodVal = dwDaGain[nChNum];
							fVoltage_GoodVal = fVoltageVal;
						}

						if( fVoltageVal < 4.5 || fVoltageVal > 5.5){
							if( fVoltageVal < 4.5 ){
								printf("検査品のch%d からのD/A出力値が低すぎます。\a\n", nChNum);
							}
							else{
								printf("検査品のch%d からのD/A出力値が高すぎます。\a\n", nChNum);
							}
							printf("出力値(0x3FFF), 調整基準値(5.0 V), マルチメータ値(%f V)\n", fVoltageVal);
							printf("hit any key.\a\n");
							HitAnyKeyWait();
							bRetCont = FALSE;
							break;
						}
						else if( fVoltageVal > 5.0 ){
							//入力値が5Vを超えたら元に戻す（今回bitをOFFする）
							dwDaGain[nChNum] = dwTemp;
						}
						else if( ii == 11 ){
							//----------------------------------------------------------------------
							//★ SP_UPER_IF_BOARDでは今のところ5V以下の基板が無いのでこの処理は保留
							//----------------------------------------------------------------------
							//最初のビット（11ビット目）ONで、入力値が5Vを超えない場合は、
							//最大値0x0fffを設定後、値が変化したビットのみを有効にする
							printf("[DebInfo]0xFFF Output Value = %fV (<5V)\n", fVoltageVal);
							//if( DaGainAdj_1(forMultiMaterHandles, nChNum, &wDoVal, &fCurVal) != TRUE ){
								printf("何かキーを押してください。\a\n");
								HitAnyKeyWait();
								bRetCont = FALSE;		//5Vが出せない基板は今のところエラーにしておく
							//}
							break;
						}

						if( ii == 0 ){	//最終bitのとき
							dwDaOffset[nChNum] = dwDaOffset_GoodVal;
							dwDaGain[nChNum] = dwDaGain_GoodVal;
							if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//変更後の値に更新する
								printf("[D/A ch%d] Gain補正値の更新に失敗しました。\a\a\n", nChNum);
								bRetCont = FALSE;
								break;
							}

							//マルチメータから最終の電圧値を取得する
							Sleep_Cnt(1000);
							if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
								printf("電圧値を取得できませんでした。何かキーを押してください。\a\n");
								bRetCont = FALSE;
								break;
							}
						}
					}	//for end (bit shift)

					wDaGain[nChNum] = (WORD)(dwDaGain[nChNum] & 0x0fff);
					printf("(ch%d) D/A Gain[0x%X] AdjOut[%fV]\n", nChNum, wDaGain[nChNum], fVoltageVal);
					fDaGainMultiMeter[nChNum] = fVoltageVal;
				}

/*
				dwTemp = dwDaGain[nChNum];
				while(ForeverLoop){
					printf("[ch%d(0x%04X)] 5.0VになるようにGain値を調整してください（終了:q）=>", nChNum, dwDaGain[nChNum]);
					KeyStrInputWait(strwk);
					if( strwk[0] == 'q' || strwk[0] == 'Q' ){
						break;
					}
					else{
						dwTemp = (DWORD)strtol(strwk, &endptr, 16);
						//if( dwTemp > 0 && dwTemp <= 0x0FFF ){
						if( dwTemp >= 0 && dwTemp <= 0x0FFF ){
							dwDaGain[nChNum] = dwTemp;
							if( dwDaOffset[nChNum] >= 0x0FFF ){
								dwDaOffset[nChNum] = 0x400;
							}
							if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//変更後の値に更新する
								printf("[D/A ch%d] Gain補正値の更新に失敗しました。\a\a\n", nChNum);
							}
						}
					}
				}
*/
				if( bRetCont == TRUE ){
					printf("\nD/A ch%dの調整を終了しますか？ (y or n) -->", nChNum);
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						break;
					}
					else if( nInputVal == 0x1B ){
						bRetCont = FALSE;
						break;
					}
				}
				else{
					break;
				}
				if( bRetCont != TRUE ) break;
			}	//調整リトライ用whileの終わり
			if( bRetCont != TRUE ) break;
		}	//D/A ch数分繰り返す

		//結果を出力する
		if( bRetCont == TRUE ){
			for( nChNum=0; nChNum<SP_XXXX_DACH_MAX; nChNum++ ){
				LogPrintfCsv("SPxxxx_DaLog.csv", "--------------------------------\n");
				LogPrintfCsv("SPxxxx_DaLog.csv", "ch, Offset(Old), Gain(Old), Offset(New), Gain(New)\n");
				LogPrintfCsv("SPxxxx_DaLog.csv", "%d, %03lX, %03lX, %03lX, %03lX\n",
												nChNum,
												dwDaOffset_OldVal[nChNum], dwDaGain_OldVal[nChNum],
												dwDaOffset[nChNum], dwDaGain[nChNum]);
			}
		}

		if( bRetCont == TRUE ){
			//printf("\nEEPROMにD/A補正値を保存しますか？ (y or n) -->");
			//nInputVal = KeyInputWait();
			//printf("\n");
			//if( nInputVal == 'y' || nInputVal == 'Y' || nInputVal == 0x0d ){
				printf("\n-----------------------------------------\n");
				printf("EEPROM保存実行中...\n");
				bRetCont = TRUE;
				if( RS_SpUpdateEeprom(SP_XXXX_DACH_MAX) == TRUE ){		//EEPROM保存実行
					printf("EEPROM保存が完了しました。\n\n");
					printf("\n-----------------------------------------\n");
					printf("EEPROMに保存されたデータを確認します。検査品の電源を再投入してください。\n");
					printf("準備ができたら何かキーを押してください。");
					KeyInputWait();
					printf("\n");

					if( RS_SpSetDaOffsetGain(dwDaOffsetTemp, dwDaGainTemp, dwDaOffsetTemp, dwDaGainTemp) == TRUE ){	//補正値の読出しのみ実行
						LogPrintfCsv("SPxxxx_DaLog.csv", " <<< EEPROM SAVE >>>\n");
						for( nChNum=0; nChNum<SP_XXXX_DACH_MAX; nChNum++ ){
							if( (dwDaOffset[nChNum] == 0xffff || dwDaGain[nChNum] == 0xffff) ||
								(dwDaOffsetTemp[nChNum] == dwDaOffset[nChNum] && dwDaGainTemp[nChNum] == dwDaGain[nChNum]) )
							{
								LogPrintfCsv("SPxxxx_DaLog.csv", " [OK]:(ch%d) Offset W[0x%04X] R[0x%04X], GAIN W[0x%04X] R[0x%04X]\n",
											nChNum, dwDaOffset[nChNum], dwDaOffsetTemp[nChNum],
											dwDaGain[nChNum], dwDaGainTemp[nChNum]);
							}
							else{
								//DA出力補正値が一致しません。
								bRetCont = FALSE;
								LogPrintfCsv("SPxxxx_DaLog.csv", " [NG]:(ch%d) Offset W[0x%04X] R[0x%04X], GAIN W[0x%04X] R[0x%04X]\n",
											nChNum, dwDaOffset[nChNum], dwDaOffsetTemp[nChNum],
											dwDaGain[nChNum], dwDaGainTemp[nChNum]);
							}
						}
					}
					else{
						printf("DA出力補正値が取得できません。\a\a\n");
						bRetCont = FALSE;
					}
				}
			//}
		}
	}
	else{
		//LogPrintfCsv("SPxxxx_DaLog.csv", "\n");
		printf("[D/A] 出力補正値の取得に失敗しました。\a\a\n");
		bRetCont = FALSE;
		//break;
	}

	/////////////////////////////////////////////////////////////////////
	//D/A出力調整終了処理
	RS_SpNormalMode();

	//マルチメータ通信リソースのハンドルを閉じる
	if( forMultiMaterHandles != NULL ){
		End_HP34401A(forMultiMaterHandles);
	}

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv("SPxxxx_DaLog.csv", 
		"■■■ [%s] D/A出力調整 %s %s FINISH ■■■\n", m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

//////////////////////////////////////////////////////////////
//SAVENETを利用した入出力テスト
//////////////////////////////////////////////////////////////

////////////////////////////////////////////
//SP_UPPER_IF_BOARD 自動でできないDIO検査
BOOL SPxxxxDioChkManualUpper1()
{
	return SPxxxxDioChkManualUpper2();
	//return TRUE;
}

////////////////////////////////////////////
//SP_UPPER_IF_BOARD2 自動でできないDIO検査
BOOL SPxxxxDioChkManualUpper2()
{
	BOOL	bRetCont=TRUE;
	int		nInputVal = -1;
	DWORD	dwInData[3];
	DWORD	dwInData_old[3];
	DWORD	dwStartTick;
	BOOL	bTestCont = TRUE;
	int		ii, jj, kk;
	DWORD	dwIN7Val, dwIN8Val, dwIN9Val;
	int		TestOkCount;
	BOOL	bChkOK;

	///////////////////////////////////////
	//IN3-8入力チェック
	while( bRetCont == TRUE ){
		printf("\n---------------------------------------------------------\n");
		printf(" ＜IN3-8入力検査開始＞\n" );
		printf(" CN34のIN3_8入力ボタンを押し続けてください\n" );
		printf("-----------------------------------------------------------\n");
		dwStartTick = GetTickCount();
		while(1){
			if(_kbhit()){
				_getch();
				break;
			}
			else{
				DATAin(BoardId, SP_XXXX_SW1_START_ADR+2, (LPINT*)&dwInData[0]);
				if( dwInData_old[0] != dwInData[0] ){
					dwInData_old[0] = dwInData[0];
					printf("\rIN3_8[%d]\r", ((dwInData[0] & 0x0100) ? 1 : 0) );
					if( (dwInData[0] & 0x0100) != 0 ){
						break;
					}
				}
			}
			Sleep_Cnt(10);
		}
		printf("\n");

		if( bRetCont == TRUE ){
			dwStartTick = GetTickCount();
			do{
				DATAin(BoardId, SP_XXXX_SW1_START_ADR+2, (LPINT*)&dwInData[0]);
				if( (dwInData[0] & 0x0100) == 0 ){
					bRetCont = FALSE;
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"IN3-8入力エラー(0x%04X)\n", (WORD)dwInData[0]);
					break;
				}
				Sleep_Cnt(5);
			}while((DWORD)(GetTickCount()-dwStartTick) < 200);
		}

		if( bRetCont == TRUE ){
			printf("\n----------------------------------------------\n");
			printf(" IN3_8入力ボタンを操作してCN33,CN34の緑LED点灯・消灯状態を目視確認してください\n" );
			printf(" ボタン押下中は点灯し、ボタンから手を離すと消灯しますか？(y or n)-->\a" );
			//printf(" IN3_8入力ボタンの押下中はCN33,CN34の緑LEDが点灯していますか？(y or n) -->\n" );
			nInputVal = KeyInputWaitYorN();
			if( nInputVal != 'y' ){
				printf("\n");
				bRetCont = FALSE;
				LogPrintfCsv("SPxxxx_DioLog.csv", "IN3-8(SW_ON)入力ON/OFF連動テストエラー(LED_ON,LIGHT_ON出力エラー)\n");
			}
			else{
				printf("\n");
			}
		}
		
		//if( bRetCont == TRUE ){
		//	printf("\n----------------------------------------------\n");
		//	printf(" ボタンから手を離すと、CN33,CN34の緑LEDは消灯しましたか？(y or n) -->\n" );
		//	nInputVal = KeyInputWait();
		//	if( nInputVal != 'y' && nInputVal != 'Y' && nInputVal != 0x0d ){
		//		printf(" n\n\n");
		//		bRetCont = FALSE;
		//		LogPrintfCsv("SPxxxx_DioLog.csv", "IN3-8(SW_ON)入力OFF連動テストエラー(LED_ON,LIGHT_ON出力エラー)\n");
		//	}
		//	else{
		//		printf(" y\n\n");
		//	}
		//}	
			
		if( bRetCont == TRUE ){
			printf("\n----------------------------------------------\n");
			LogPrintfCsv("SPxxxx_DioLog.csv", "IN3_8 入力検査OK\n");
			printf("----------------------------------------------\n\n");
		}
		else{
			printf("\n----------------------------------------------\n");
			LogPrintfCsv("SPxxxx_DioLog.csv", "IN3_8 入力検査NG\n");
			printf("----------------------------------------------\n\n");
			printf("再度、IN3_8 入力テストを行いますか？(y or n) -->");
			nInputVal = KeyInputWaitYorN();
			if( nInputVal == 'y' ){
				bRetCont = TRUE;
				continue;
			}
		}
		break;
	}

	if( m_nIniSpUpperBoardType == 1 ){	//Upper2?
		///////////////////////////////////////
		//IN7-e,f IN8-e,f IN9-e,f(ALM_A, ALM_B)入力チェック、ZEROSET出力チェック
		//if(0){
		while( bRetCont == TRUE ){
			printf("\n---------------------------------------------------------\n");
			printf(" ＜ALM_A, ALM_B, SIG 入力検査開始＞\n" );
			printf("-----------------------------------------------------------\n");
			
			for( ii=0; ii<2; ii++ ){	//DSW ON,OFF操作による処理の切り替え
				if( bRetCont == TRUE ){
					printf("\n\n");
					printf("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■\n");
					printf("  [テスト準備]\a\n");
					printf("  基板上のDSW1～DSW3を%sにした後、指示に従って操作してください\n", ((ii==0) ? "ON": "OFF"));
					printf("■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■\n\n");
					//nInputVal = KeyInputWait();
					//if ( nInputVal == 0x1B ){
					//	bRetCont = FALSE;
					//}

					for( jj=0; jj<3; jj++ ){	//SWグループ毎に処理
						if( bRetCont == TRUE ){
							TestOkCount = 0;
							while((TestOkCount<2) && (bRetCont == TRUE)){	//同じ操作を2回繰り返す
								kk=0;
								while(kk<3){		//ボタン毎に処理
								//for( kk=0; kk<3; kk++ ){	//ボタン毎に処理
									//期待する入力値dwIN7Val～dwIN9Valにセットする
									switch(jj){
									case 0:	//1番目のSWグループ
										if( ii == 0 ){	//DSWがONのとき
											dwIN7Val = 0xC000;
											dwIN8Val = 0x0000;
											dwIN9Val = 0x0000;
										}
										else{			//DSWがOFFのとき
											switch(kk){
											case 0:
												dwIN7Val = 0x4000;
												dwIN8Val = 0x0000;
												dwIN9Val = 0x0000;
												break;
											case 1:
												dwIN7Val = 0x8000;
												dwIN8Val = 0x0000;
												dwIN9Val = 0x0000;
												break;
											case 2:
												dwIN7Val = 0x4000;
												dwIN8Val = 0x0000;
												dwIN9Val = 0x0000;
												break;
											}
										}
										break;
									case 1:	//2番目のSWグループ
										if( ii == 0 ){	//DSWがONのとき
											dwIN7Val = 0x0000;
											dwIN8Val = 0xC000;
											dwIN9Val = 0x0000;
										}
										else{			//DSWがOFFのとき
											switch(kk){
											case 0:
												dwIN7Val = 0x0000;
												dwIN8Val = 0x4000;
												dwIN9Val = 0x0000;
												break;
											case 1:
												dwIN7Val = 0x0000;
												dwIN8Val = 0x8000;
												dwIN9Val = 0x0000;
												break;
											case 2:
												dwIN7Val = 0x0000;
												dwIN8Val = 0x4000;
												dwIN9Val = 0x0000;
												break;
											}
										}
										break;
									case 2:	//3番目のSWグループ
										if( ii == 0 ){	//DSWがONのとき
											dwIN7Val = 0x0000;
											dwIN8Val = 0x0000;
											dwIN9Val = 0xC000;
										}
										else{			//DSWがOFFのとき
											switch(kk){
											case 0:
												dwIN7Val = 0x0000;
												dwIN8Val = 0x0000;
												dwIN9Val = 0x4000;
												break;
											case 1:
												dwIN7Val = 0x0000;
												dwIN8Val = 0x0000;
												dwIN9Val = 0x8000;
												break;
											case 2:
												dwIN7Val = 0x0000;
												dwIN8Val = 0x0000;
												dwIN9Val = 0x4000;
												break;
											}
										}
										break;
									}

									DATAin(BoardId, SP_XXXX_SW1_START_ADR+6, (LPINT*)&dwInData[0]);
									DATAin(BoardId, SP_XXXX_SW1_START_ADR+7, (LPINT*)&dwInData[1]);
									DATAin(BoardId, SP_XXXX_SW1_START_ADR+8, (LPINT*)&dwInData[2]);
									if( (dwInData[0] & 0xC000) != 0 || (dwInData[1] & 0xC000) != 0 || (dwInData[2] & 0xC000) != 0 ){
										//printf(" SWから手を離してください\n");
										while(1){
											if(_kbhit()){
												_getch();
												break;
											}
											DATAin(BoardId, SP_XXXX_SW1_START_ADR+6, (LPINT*)&dwInData[0]);
											DATAin(BoardId, SP_XXXX_SW1_START_ADR+7, (LPINT*)&dwInData[1]);
											DATAin(BoardId, SP_XXXX_SW1_START_ADR+8, (LPINT*)&dwInData[2]);
											if( (dwInData[0] & 0xC000) == 0 && (dwInData[1] & 0xC000) == 0 && (dwInData[2] & 0xC000) == 0 ){
												break;
											}
											Sleep_Cnt(10);
										}
									}

									printf("\n---------------------------------------------------------\n");
									printf(" [テスト%d回目] %d番目の%sボタンを押し続けてください\n", TestOkCount+1, jj+1, ((kk==0) ? "ALM_A": ((kk==1) ? "ALM_B" : "DIN")));
									printf("-----------------------------------------------------------\n");
									while(1){
										if(_kbhit()){
											_getch();
											break;
										}
										DATAin(BoardId, SP_XXXX_SW1_START_ADR+6, (LPINT*)&dwInData[0]);
										DATAin(BoardId, SP_XXXX_SW1_START_ADR+7, (LPINT*)&dwInData[1]);
										DATAin(BoardId, SP_XXXX_SW1_START_ADR+8, (LPINT*)&dwInData[2]);
										if( ((dwInData[0] & 0xC000) == dwIN7Val) &&
											((dwInData[1] & 0xC000) == dwIN8Val) &&
											((dwInData[2] & 0xC000) == dwIN9Val) ){
											//Sleep_Cnt(300);
											Sleep_Cnt(50);
											break;
										}
										Sleep_Cnt(10);
									}

									bChkOK = TRUE;
									dwStartTick = GetTickCount();
									do{
										DATAin(BoardId, SP_XXXX_SW1_START_ADR+6, (LPINT*)&dwInData[0]);
										DATAin(BoardId, SP_XXXX_SW1_START_ADR+7, (LPINT*)&dwInData[1]);
										DATAin(BoardId, SP_XXXX_SW1_START_ADR+8, (LPINT*)&dwInData[2]);
										if( ((dwInData[0] & 0xC000) != dwIN7Val) ||
											((dwInData[1] & 0xC000) != dwIN8Val) ||
											((dwInData[2] & 0xC000) != dwIN9Val) ){
											LogPrintfCsv("SPxxxx_DioLog.csv",
												"[SW%d] DSW_%s時 %s入力エラー IN7(0x%04X) IN8(0x%04X) IN9(0x%04X)\n",
												jj+1, ((ii==0) ? "ON" : "OFF"),
												((kk==0) ? "ALM_A": ((kk==1) ? "ALM_B" : "DIN")),
												(WORD)dwInData[0], (WORD)dwInData[1], (WORD)dwInData[2]);
											printf("もう一度検査しますか？(y or n) -->");
											nInputVal = KeyInputWaitYorN();
											if( nInputVal != 'y' ){
												bRetCont = FALSE;
											}
											bChkOK=FALSE;
											break;
										}
										Sleep_Cnt(5);
									//}while((DWORD)(GetTickCount()-dwStartTick) < 200);
									}while((DWORD)(GetTickCount()-dwStartTick) < 50);

									if( bChkOK==TRUE ){
										kk++;
										printf(" SW 押下を検出しました\n");
									}
									if( bRetCont != TRUE ){
										break;
									}
								} //ボタン毎に処理

								if( bRetCont == TRUE ){
									TestOkCount++;
								}

							}	//同じテストを２回繰り返す
						}
					}
				}
			}
			if( bRetCont == TRUE ){
				printf("\n----------------------------------------------\n");
				LogPrintfCsv("SPxxxx_DioLog.csv", "ALM_A, ALM_B, SIG 入力検査 OK\n");
				printf("----------------------------------------------\n");
			}
			else{
				printf("\n----------------------------------------------\n");
				LogPrintfCsv("SPxxxx_DioLog.csv", "ALM_A, ALM_B, SIG 入力検査 NG\n");
				printf("----------------------------------------------\n");
				printf("再度、ALM_A, ALM_B, SIG 入力検査を行いますか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					bRetCont = TRUE;
					continue;
				}
			}
			break;
		}
	
		while( bTestCont == TRUE ){
			printf("\n---------------------------------------------------------\n");
			printf(" ＜ZEROSET 出力検査開始＞\n" );
			printf("-----------------------------------------------------------\n");
			for( jj=0; jj<3; jj++ ){	//SWグループ毎に処理
				if( bRetCont == TRUE ){
					DATAout(BoardId, SP_XXXX_SW1_START_ADR+6, 0x0000);
					DATAout(BoardId, SP_XXXX_SW1_START_ADR+7, 0x0000);
					DATAout(BoardId, SP_XXXX_SW1_START_ADR+8, 0x0000);
					DATAout(BoardId, (WORD)(SP_XXXX_SW1_START_ADR+6+jj), 0x8000);
					printf("%d番目のZEROSET(緑LED)のみ点灯していますか？(y or n) -->", jj+1);
					nInputVal = KeyInputWaitYorN();
					if( nInputVal != 'y' ){
						bRetCont = FALSE;
						LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET (ADR%d) 出力目視検査 NG\n", SP_XXXX_SW1_START_ADR+6+jj);
					}
				}
			}
			if( bRetCont == TRUE ){
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+6, 0x8000);
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+7, 0x8000);
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+8, 0x8000);
				printf("ZEROSET(緑LED)は全3個とも点灯していますか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal != 'y' ){
					bRetCont = FALSE;
					LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET 出力目視検査(ALL ON) NG\n");
				}
			}
			if( bRetCont == TRUE ){
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+6, 0x0000);
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+7, 0x0000);
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+8, 0x0000);
				printf("ZEROSET(緑LED)は全3個とも消灯しましたか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal != 'y' ){
					bRetCont = FALSE;
					LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET 出力目視検査(ALL OFF) NG\n");
				}
			}

			if( bRetCont == TRUE ){
				printf("\n----------------------------------------------\n");
				LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET 出力目視検査 OK\n");
				printf("----------------------------------------------\n");
			}
			else{
				printf("\n----------------------------------------------\n");
				LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET 出力目視検査 NG\n");
				printf("----------------------------------------------\n");
				printf("再度、ZEROSET 出力検査を行いますか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					bRetCont = TRUE;
					continue;
				}
			}
			break;
		}
	}
	if( bRetCont == TRUE ){
		LogPrintfCsv("SPxxxx_DioLog.csv", "手動DIO検査OK\n");
		printf("\n");
	}
	else{
		LogPrintfCsv("SPxxxx_DioLog.csv", "手動DIO検査NG\n");
		printf("\n\a");
	}

	return bRetCont;
}

////////////////////////////////////////////
//SP_UPPER_IF_BOARD SAVENETを利用したDIO検査
BOOL	SPxxxxDioChkAutoUpper1()
{
	BOOL	bRetCont=TRUE;
	BOOL	bZeroChkEnable = TRUE;
	//BOOL	bHlsStopTest = TRUE;
	//int		nInputVal;
	int		ii;

	WORD	wAnalogBitMask;
	//WORD	wTestPatten[16];
	int		nPatternCnt=0;

	/////////////////////////////////////////////////////////////////////
	//DI検査 (1点単位の入力検査)
	/////////////////////////////////////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n\n");
		printf("-----------------------------------------------\n");
		printf("SP_UPPER_IF_BOARD: DIO自動検査を開始しました。\n");
		for( ii=TEST_PATTERN0; ii<(TEST_PATTERN0 + UPPER1_TEST_PATTERN_NUM); ii++ ){
			bZeroChkEnable = TRUE;
			if( ii == TEST_PATTERN2 || ii == TEST_PATTERN8 ){
				bZeroChkEnable = FALSE;	//パターン8では複数ON入力があるため他AdrのZeroチェックはしない
			}
			else{
				bZeroChkEnable = TRUE;
			}

			if( (m_wUpper1TestPattern[ii].wInAdr >= SP_XXXX_SW1_START_ADR+3) &&
				(m_wUpper1TestPattern[ii].wInAdr <= SP_XXXX_SW1_START_ADR+8) ){
				wAnalogBitMask = 0xc000;	//アナログ入力アドレスは下位14bitを0でマスクする
			}
			else{
				wAnalogBitMask = DIO_TEST_ALL_BIT;
			}

			if( m_wUpper1TestPattern[ii].wTestBitNum > 0 ){
				if( bRetCont == TRUE ){
					bRetCont = DioAutoTestSelPattern(m_wUpper1TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);
				}
				else{
					DioAutoTestSelPattern(m_wUpper1TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);	//NG発生時も引き続き検査を続行
					//break;
				}
			}
		}
	}

	return bRetCont;
}

////////////////////////////////////////////
//SP_UPPER_IF_BOARD2 SAVENETを利用したDIO検査
BOOL	SPxxxxDioChkAutoUpper2()
{
	BOOL	bRetCont=TRUE;
	BOOL	bZeroChkEnable = TRUE;
	//BOOL	bHlsStopTest = TRUE;
	int		nInputVal;
	int		ii;

	WORD	wAnalogBitMask;
	WORD	wTestPatten[16];
	int		nPatternCnt=0;

	/////////////////////////////////////////////////////////////////////
	//HLS通信停止時の出力OFFテスト
	printf("\n");
	printf("-----------------------------------------------\n");
	printf(" HLS通信停止時の出力OFF確認\n");
	printf("-----------------------------------------------\n");
	/////////////////////////////////////////////////////////////////////
	//if( bRetCont != TRUE ){
	//	printf("\nHLS通信停止時の出力OFFテストを実施しますか？(y or n) -->");
	//	nInputVal = KeyInputWait();
	//	if( nInputVal != 'y' && nInputVal != 'Y' && nInputVal != 0x0d ){
	//		printf(" n\n");
	//		bHlsStopTest = FALSE;	//HLS通信停止時の出力OFF検査を実施しない
	//	}
	//	else{
	//		printf(" y\n");
	//	}
	//}
	//if( bHlsStopTest == TRUE ){
		//CN71 OUT3_0～OUT3_7 出力OFFテスト
		memset(wTestPatten, 0, sizeof(wTestPatten));
		wTestPatten[nPatternCnt++] = TEST_PATTERN9;
		while(1){
			if( HlsOutStopTest(m_wUpper2TestPattern, wTestPatten, nPatternCnt, 300) != TRUE ){
				printf("\n検査を中止しますか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal != 'y' ){
					continue;
				}
				bRetCont = FALSE;
			}
			break;
		}
	//}

	/////////////////////////////////////////////////////////////////////
	//DI検査 (1点単位の入力検査)
	/////////////////////////////////////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n\n");
		printf("-----------------------------------------------\n");
		printf("SP_UPPER_IF_BOARD2: DIO自動検査を開始しました。\n");
		for( ii=TEST_PATTERN0; ii<TEST_PATTERN0+UPPER2_TEST_PATTERN_NUM; ii++ ){
			if( ii == TEST_PATTERN2 || ii == TEST_PATTERN8 /*|| ii == TEST_PATTERN9 || ii == TEST_PATTERN15 || ii == TEST_PATTERN16*/){
				bZeroChkEnable = FALSE;	//パターン2, 8では複数ON入力があるため他AdrのZeroチェックはしない
			}
			else{
				bZeroChkEnable = TRUE;
			}

			if( (m_wUpper2TestPattern[ii].wInAdr >= SP_XXXX_SW1_START_ADR+3) &&
				(m_wUpper2TestPattern[ii].wInAdr <= SP_XXXX_SW1_START_ADR+8) ){
				wAnalogBitMask = 0xc000;	//アナログ入力アドレスは下位14bitを0でマスクする
			}
			else{
				wAnalogBitMask = DIO_TEST_ALL_BIT;
			}

			if( m_wUpper2TestPattern[ii].wTestBitNum > 0 ){
				if( bRetCont == TRUE ){
					bRetCont = DioAutoTestSelPattern(m_wUpper2TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);
				}
				else{
					DioAutoTestSelPattern(m_wUpper2TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);	//NG発生時も引き続き検査を続行
					//break;
				}
			}
		}
	}

	return bRetCont;
}

/***
///////////////////////////////////////////////
// SP_UPPER_IF_BOARD2 アラーム入力テスト（未使用）
BOOL	UpperAlarmInOutTest(WORD wOutAdr1, WORD wOutData1, WORD wOutAdr2, WORD wOutData2, 
							WORD wInAdr1, WORD wChkData1, WORD wInAdr2, WORD wChkData2, DWORD dwChkTime)
{
	DWORD	dwStartTick;
	DWORD	dwInData1, dwInData2;
	BOOL	bRet = TRUE;
	
	//テストデータを出力する
	DATAout(BoardId, wOutAdr1, wOutData1);
	DATAout(BoardId, wOutAdr2, wOutData2);

	Sleep_Cnt(50);

	//入力値が正しく取得できることを確認する
	dwStartTick = GetTickCount();
	do{
		DATAin(BoardId, wInAdr1, (LPINT*)&dwInData1);
		DATAin(BoardId, wInAdr2, (LPINT*)&dwInData2);
		if( (WORD)dwInData1 != wChkData1 || (WORD)dwInData2 != wChkData2 ){
			bRet = FALSE;
			LogPrintfCsv("SPxxxx_DioLog.csv",
				"Out_Adr%d(0x%04X),Out_Adr%d(0x%04X) : AlmIn<Chk>%d(0x%04X<0x%04X>),AlmIn<Chk>%d(0x%04X<0x%04X>)\n", 
					wOutAdr1, wOutData1, wOutAdr2, wOutData1,
					wInAdr1, (int)dwInData1, wChkData1, wInAdr2, (int)dwInData2, wChkData2);
			break;
		}
		Sleep_Cnt(5);
	}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);

	return bRet;
}
***/
////////////////////////////////////////////
//SP_FRONT_IF_BOARD2 SAVENETを利用したDIO検査
BOOL	SPxxxxDioChkAutoFront2()
{
	BOOL	bRetCont=TRUE;
	BOOL	bZeroChkEnable;
	//BOOL	bHlsStopTest = TRUE;
	int		nInputVal;
	int		ii;

	WORD	wAnalogBitMask;
	WORD	wTestPatten[16];
	int		nPatternCnt=0;

	/////////////////////////////////////////////////////////////////////
	//HLS通信停止時の出力OFFテスト
	/////////////////////////////////////////////////////////////////////
	printf("\n");
	printf("-----------------------------------------------\n");
	printf(" HLS通信停止時の出力OFF確認\n");
	printf("-----------------------------------------------\n");
	//if( bRetCont != TRUE ){
	//	printf("\nHLS通信停止時の出力OFFテストを実施しますか？(y or n) -->");
	//	nInputVal = KeyInputWait();
	//	if( nInputVal != 'y' && nInputVal != 'Y' && nInputVal != 0x0d ){
	//		printf(" n\n");
	//		bHlsStopTest = FALSE;	//HLS通信停止時の出力OFF検査を実施しない
	//	}
	//	else{
	//		printf(" y\n");
	//	}
	//}
	//if( bHlsStopTest == TRUE ){
		memset(wTestPatten, 0, sizeof(wTestPatten));
		//CN61 OUT1_0～OUT1_f 出力OFFテスト
		wTestPatten[nPatternCnt++] = TEST_PATTERN10;
		//CN33 OUT2_0～OUT2_3 出力OFFテスト
		wTestPatten[nPatternCnt++] = TEST_PATTERN15;
		//CN41 OUT3_0～OUT3_b 出力OFFテスト
		wTestPatten[nPatternCnt++] = TEST_PATTERN16;
		//CN42 OUT3_0～OUT3_b 出力OFFテスト
		wTestPatten[nPatternCnt++] = TEST_PATTERN18;

		while(1){
			if( HlsOutStopTest(m_wFront2TestPattern, wTestPatten, nPatternCnt, 300) != TRUE ){
				printf("\n検査を中止しますか？(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal != 'y' ){
					continue;
				}
				bRetCont = FALSE;
			}
			break;
		}
	//}

	/////////////////////////////////////////////////////////////////////
	//DI検査 (1点単位の入力検査)
	/////////////////////////////////////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n\n");
		printf("-----------------------------------------------\n");
		printf("SP_FRONT_IF_BOARD2: DIO自動検査を開始しました。\n");
		for( ii=TEST_PATTERN0; ii<TEST_PATTERN0+FRONT_TEST_PATTERN_NUM; ii++ ){
			if( ii == TEST_PATTERN2 || ii == TEST_PATTERN8 || ii == TEST_PATTERN9 || ii == TEST_PATTERN15 || ii == TEST_PATTERN16){
				bZeroChkEnable = FALSE;	//パターン2, 8, 9, 15, 16では複数ON入力があるため他AdrのZeroチェックはしない
			}
			else{
				bZeroChkEnable = TRUE;
			}

			if( (m_wFront2TestPattern[ii].wInAdr >= SP_XXXX_SW1_START_ADR+3) &&
				(m_wFront2TestPattern[ii].wInAdr <= SP_XXXX_SW1_START_ADR+6) ){
				wAnalogBitMask = 0xc000;	//アナログ入力アドレスは下位14bitを0でマスクする
			}
			else{
				wAnalogBitMask = DIO_TEST_ALL_BIT;
			}
			
			if( bRetCont == TRUE ){
				bRetCont = DioAutoTestSelPattern(m_wFront2TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);
				if( bRetCont != TRUE ){
					printf("\n検査を中止しますか？(y or n) -->");
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						break;
					}
				}
			}
			else{
				DioAutoTestSelPattern(m_wFront2TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);	//NG発生時も引き続き検査を続行
				//break;
			}
		}
	}
	return bRetCont;
}

///////////////////////////////////////////////
// DIO折り返しテスト
/**
BOOL DioAutoTestAllBit(WORD wSnOutAdr, WORD wSnInAdr, DWORD dwChkTime)
{
	return DioAutoTestSelBit(wSnOutAdr, 0, 15, wSnInAdr, 0, 15, dwChkTime);
}
BOOL DioAutoTestSelBit(	WORD wSnOutAdr, WORD wSnOutStart, WORD wSnOutEnd,
						WORD wSnInAdr, WORD wSnInStart, WORD wSnInEnd, DWORD dwChkTime )
{
	BOOL	bRet=TRUE;
	WORD	wOutBit;
	WORD	wInBit;
	WORD	wOutVal;
	WORD	wChkVal;
	DWORD	dwIndata;
	DWORD	dwStartTick;

	//bit walk test
	for( wOutBit=wSnOutStart, wInBit=wSnInStart; wOutBit<=wSnOutEnd; wOutBit++, wInBit++ ){
		wOutVal = 0x0001 << wOutBit;
		wChkVal = 0x0001 << wInBit;
		DATAout(BoardId, wSnOutAdr, wOutVal);
		Sleep_Cnt(50);

		//入力値が正しく取得できることを確認する
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, wSnInAdr, (LPINT*)&dwIndata);
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//データ不一致エラー
				LogPrintfCsv("SPxxxx_DioLog.csv",
					" Out_Adr%d-b%d(0x%04X):In_Adr%d-b%d(0x%04d) [NG:データ不一致]\n", 
						wSnOutAdr, wOutBit, wOutVal, wSnInAdr, wInBit, (int)dwIndata);
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);

		if( bRet != TRUE ){
			break;
		}
	}

	//全点ON確認
	if( bRet == TRUE ){
		wOutVal = 0x0000;
		wChkVal = 0x0000;
		for( wOutBit=wSnOutStart, wInBit=wSnInStart; wOutBit<=wSnOutEnd; wOutBit++, wInBit++ ){
			wOutVal = wOutVal | (0x0001 << wOutBit);
			wChkVal = wChkVal | (0x0001 << wInBit);
		}
		DATAout(BoardId, wSnOutAdr, wOutVal);
		Sleep_Cnt(50);

		//入力値が正しく取得できることを確認する
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, wSnInAdr, (LPINT*)&dwIndata);
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//データ不一致エラー
				LogPrintfCsv("SPxxxx_DioLog.csv",
					" Out_Adr%d-b%d(0x%04X):In_Adr%d-b%d(0x%04d) [NG:データ不一致]\n", 
						wSnOutAdr, wOutBit, wOutVal, wSnInAdr, wInBit, (int)dwIndata);
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);
	}

	//全点OFF確認
	if( bRet == TRUE ){
		wOutVal = 0x0000;
		wChkVal = 0x0000;
		DATAout(BoardId, wSnOutAdr, wOutVal);
		Sleep_Cnt(50);

		//入力値が正しく取得できることを確認する
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, wSnInAdr, (LPINT*)&dwIndata);
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//データ不一致エラー
				LogPrintfCsv("SPxxxx_DioLog.csv",
					" Out_Adr%d-b%d(0x%04X):In_Adr%d-b%d(0x%04d) [NG:データ不一致]\n", 
						wSnOutAdr, wOutBit, wOutVal, wSnInAdr, wInBit, (int)dwIndata);
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);
	}
	return bRet;
}
**/

///////////////////////////////////////////////
// DIO 他アドレスの入力がON/OFF変化しないことを確認する
BOOL OtherUnitInputZeroChk( WORD wStartAdr, WORD wEndAdr, WORD wSkipAdr )
{
	DWORD	termerr;            // ターミナル通信状態
	DWORD	dwIndata;
	WORD	ii;
	BOOL	bRet = TRUE;

	for( ii=wStartAdr; ii<=wEndAdr; ii++ ){
		if(ii != wSkipAdr){
			DATAin(BoardId, ii, (LPINT*)&dwIndata);
			if( dwIndata != 0 ){
				bRet = FALSE;
				LogPrintfCsv("SPxxxx_DioLog.csv",
							"HLS Adr.%d 他アドレスデータ０チェックエラー(0x%04X)\n", ii, (WORD)dwIndata);
			}

			termerr = 0xffff;
			TERMchk(BoardId, ii, (LPINT*)&termerr);
			if(termerr & 0x7C00){
				bRet = FALSE;		//HLS通信エラー
				LogPrintfCsv("SPxxxx_DioLog.csv",
							"HLS Adr.%d 通信エラー(0x%04X)\n", ii, termerr);
			}
		}
	}
	
	return bRet;
}

/////////////////////////////////////////////////////////////////////
//HLS通信停止時の出力OFFテスト
BOOL HlsOutStopTest( DIO_AUTO_TEST_TABLE *iotbl, WORD *wPatternNo, int nPatternCnt, DWORD dwChkTime )
{
	BOOL	bRet=TRUE;
	WORD	ii, jj;
	WORD	wOutVal[16];
	WORD	wChkVal[16];
	DWORD	dwIndata[16];
	DWORD	dwStartTick;
	DWORD	OutTermChk;		// 出力側ターミナル通信状態
	DWORD	InTermChk;		// 入力側ターミナル通信状態
	DWORD	OutCmpVal;
	DWORD	InCmpVal;
	int		nInputVal;
	BOOL	bChkFlg;

	//前回検査でエラーのとき検査開始時に出力ONのままになるときがあるので、出力値を全クリア
	SnOutputInit(FALSE);

	//(前処理)
	//SP基板側の出力をONすることで、SAVENET折り返し入力値もONすることを確認する
	if( bRet == TRUE ){
		for( jj=0; jj<nPatternCnt; jj++ ){
			wOutVal[jj] = 0x0000;
			wChkVal[jj] = 0x0000;
			for( ii=0; ii<iotbl[wPatternNo[jj]].wTestBitNum; ii++ ){
				wOutVal[jj] = wOutVal[jj] | (0x0001 << iotbl[wPatternNo[jj]].wOutBit[ii]);
				wChkVal[jj] = wChkVal[jj] | (0x0001 << iotbl[wPatternNo[jj]].wInBit[ii]);
			}
			DATAout(BoardId, iotbl[wPatternNo[jj]].wOutAdr, wOutVal[jj]);	//SP基板側の出力をONする
		}
		Sleep_Cnt(50);	//少しだけwait

		//入力値が安定して読み出せることを確認する
		dwStartTick = GetTickCount();
		do{
			for( jj=0; jj<nPatternCnt; jj++ ){
				DATAin(BoardId, iotbl[wPatternNo[jj]].wInAdr, (LPINT*)&dwIndata[jj]);
				if( (WORD)dwIndata[jj] != wChkVal[jj] ){
					bRet = FALSE;		//データ不一致エラー
					LogPrintfCsv("SPxxxx_DioLog.csv",
						"[HLS STOP TestPtn%d_1]  Out_Adr%d=0x%04X→In_Adr%d=0x%04X [(全点ONテストNG:)データ不一致]\n", 
							wPatternNo[jj],
							iotbl[wPatternNo[jj]].wOutAdr,
							wOutVal[jj],
							iotbl[wPatternNo[jj]].wInAdr,
							(int)dwIndata[jj]);
					break;
				}
				else{
					OutTermChk = 0xffff;
					TERMchk(BoardId, (WORD)iotbl[wPatternNo[jj]].wOutAdr, (LPINT*)&OutTermChk);
					if(OutTermChk & 0x7C00){
						bRet = FALSE;		//HLS通信エラー
						LogPrintfCsv("SPxxxx_DioLog.csv",
									"[HLS STOP TestPtn%d] HLS Adr.%d 通信エラー(0x%04X)\n", wPatternNo[jj], iotbl[wPatternNo[jj]].wOutAdr, OutTermChk);
						break;
					}

					InTermChk = 0xffff;
					TERMchk(BoardId, (WORD)iotbl[wPatternNo[jj]].wInAdr, (LPINT*)&InTermChk);
					if(InTermChk & 0x7C00){
						bRet = FALSE;		//HLS通信エラー
						LogPrintfCsv("SPxxxx_DioLog.csv",
									"[HLS STOP TestPtn%d] HLS Adr.%d 通信エラー(0x%04X)\n", wPatternNo[jj], iotbl[wPatternNo[jj]].wInAdr, InTermChk);
						break;
					}
				}
			}
			if( bRet != TRUE ){
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);
	}

	//HLS通信停止時と通信正常時の入力値を監視
	for( ii=0; ii<2; ii++ ){		//ii=0:HLS通信停止時の出力OFF確認、1:HLS通信正常時の出力ON確認
		printf("\nSP基板側のHLS通信を%sしてください\a\n", ((ii==0) ? "停止":"復旧"));
		if( ii == 0 ){
			OutCmpVal = 0x7C00;		//出力側は通信異常になるのを待つ
			InCmpVal = 0x0000;		//入力側は通信正常になるのを待つ
		}
		else{
			OutCmpVal = 0x0000;		//出力側は通信正常になるのを待つ
			InCmpVal = 0x0000;		//入力側は通信正常になるのを待つ
		}

		while(1){
			bChkFlg = TRUE;
			for( jj=0; jj<nPatternCnt; jj++ ){
				OutTermChk = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo[jj]].wOutAdr, (LPINT*)&OutTermChk);

				InTermChk = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo[jj]].wInAdr, (LPINT*)&InTermChk);

				if( ((OutTermChk & 0x7C00) == OutCmpVal) && ((InTermChk & 0x7C00) == InCmpVal) ){
					DATAin(BoardId, iotbl[wPatternNo[jj]].wInAdr, (LPINT*)&dwIndata[jj]);
					if( ii == 0 ){
						if( (WORD)dwIndata[jj] != 0x0000 ){
							printf("\rHLS停止待ち...Out(Adr%d=%04X),In(Adr%d=%04X)\r",
									iotbl[wPatternNo[jj]].wOutAdr, wOutVal[jj],
									iotbl[wPatternNo[jj]].wInAdr, (WORD)dwIndata[jj]);
							bChkFlg = FALSE;	//HLS停止後に出力がOFFしていない
							break;
						}
					}
					else{
						if( (WORD)dwIndata[jj] != wChkVal[jj] ){
							printf("\rHLS復旧待ち...Out(Adr%d=%04X),In(Adr%d=%04X)\r",
									iotbl[wPatternNo[jj]].wOutAdr, wOutVal[jj],
									iotbl[wPatternNo[jj]].wInAdr, (WORD)dwIndata[jj]);
							bChkFlg = FALSE;	//HLS通信復旧後に出力がONしていない
							break;
						}
					}
				}
				else{
					printf("\rHLS通信%s待ち...\r",((ii==0) ? "停止":"復旧"));
					bChkFlg = FALSE;			//SP側のHLS通信が(ii=0:停止,1:復旧)していない
				}

				if( _kbhit() ){
					nInputVal = _getch();
					if( nInputVal == 0x1B ){
						bRet = FALSE;			//中断操作あり
						printf("\n");
						LogPrintfCsv("SPxxxx_DioLog.csv", "[HLS STOP TestPtn%d] 中断されました\n", wPatternNo[jj]);
						break;
					}
				}
			}
			if( bChkFlg == TRUE || bRet != TRUE ){
				printf("\n");
				break;		//SP側の通信(ii=0:停止,1:復旧)を検出
			}
		}	//while(1)の終わり

		Sleep_Cnt(50);

		//////////////////////////////////////////////////////////////
		//入力値(ii=0:OFF,1:ON)が安定して読み出せることを確認する
		if( bRet == TRUE ){
			dwStartTick = GetTickCount();
			do{
				for( jj=0; jj<nPatternCnt; jj++ ){
					if( ii == 0 ){
						InCmpVal = 0x0000;				//HLS停止時は入力がOFFであることを確認する
					}
					else{
						InCmpVal = (DWORD)wChkVal[jj];	//HLS正常通信時は入力がONであることを確認する
					}
					
					DATAin(BoardId, iotbl[wPatternNo[jj]].wInAdr, (LPINT*)&dwIndata[jj]);
					if( dwIndata[jj] != InCmpVal ){
						bRet = FALSE;		//データ(ii=0:OFF,1:ON)入力エラー
						LogPrintfCsv("SPxxxx_DioLog.csv",
							"[HLS STOP TestPtn%d_3]  Out_Adr%d=0x%04X→In_Adr%d=0x%04X [NG: HLS%s時の入力データ異常]\n", 
								wPatternNo[jj],
								iotbl[wPatternNo[jj]].wOutAdr,
								wOutVal[jj],
								iotbl[wPatternNo[jj]].wInAdr,
								(int)dwIndata[jj],
								((ii==0) ? "停止":"復旧"));
						break;
					}
				}
				if( bRet != TRUE ){
					break;
				}
				Sleep_Cnt(5);
			}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);
		}

		if( bRet != TRUE ){
			break;
		}
	}	//for(ii=0:HLS通信停止時の出力OFF確認、1:HLS通信正常時の出力ON確認)の終わり

	//後処理
	for( jj=0; jj<nPatternCnt; jj++ ){
		DATAout(BoardId, iotbl[wPatternNo[jj]].wOutAdr, 0x0000);	//出力をOFFする
	}

	if( bRet == TRUE ){
		LogPrintfCsv("SPxxxx_DioLog.csv", "HLS通信停止時の出力OFF確認 <OK>\n");
	}
	else{
		LogPrintfCsv("SPxxxx_DioLog.csv", "HLS通信停止時の出力OFF確認 <NG!>\n");
	}
	return bRet;
}

///////////////////////////////////////////////
// DIO折り返しテスト(複数bit)
BOOL DioAutoTestSelPattern( DIO_AUTO_TEST_TABLE *iotbl, WORD wPatternNo, DWORD dwChkTime, BOOL bZeroChkEnable, WORD wInMaskBit )
{
	BOOL	bRet=TRUE;
	WORD	ii;
	//WORD	wOutBit;
	WORD	wOutVal;
	WORD	wChkVal;
	DWORD	dwIndata;
	DWORD	dwStartTick;
	DWORD	termerr;            // ターミナル通信状態
	BOOL	bZeroChk;
	char	cSignalNameWork[2][16];
	char	cPinNameWork[2][16];
	
	WORD	wHlsZeroTestAdrStart_1 = 0;
	WORD	wHlsZeroTestAdrEnd_1 = 0;
	WORD	wHlsZeroTestAdrStart_2 = 0;
	WORD	wHlsZeroTestAdrEnd_2 = 0;

	if( m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ){
		//#7～#10はアナログ入力につきZeroテストはしない
		wHlsZeroTestAdrStart_1 = SP_XXXX_SW1_START_ADR;		//#4
		wHlsZeroTestAdrEnd_1 = SP_XXXX_SW1_START_ADR+2;		//#6
		wHlsZeroTestAdrStart_2 = ADR_DI_48;
		wHlsZeroTestAdrEnd_2 = ADR_DI_58;
	}
	else if ( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
		//#7～#12はアナログ入力につきZeroテストはしない
		wHlsZeroTestAdrStart_1 = SP_XXXX_SW1_START_ADR;		//#4
		wHlsZeroTestAdrEnd_1 = SP_XXXX_SW1_START_ADR+2;		//#6
		wHlsZeroTestAdrStart_2 = ADR_DI_48;
		wHlsZeroTestAdrEnd_2 = ADR_DI_58;
	}
	else if ( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
		if( m_nIniSpMotherBoardType == 0 ){		//SP_MOTHER_BOARD
			//#10はアナログ入力につきZeroテストはしない
			wHlsZeroTestAdrStart_1 = SP_XXXX_SW1_START_ADR;		//#4
			wHlsZeroTestAdrEnd_1 = SP_XXXX_SW2_START_ADR+1;		//#9
		}
		else{		//SPIN_MOTHER_BOARD
			//#9はアナログ入力につきZeroテストはしない
			wHlsZeroTestAdrStart_1 = SP_XXXX_SW1_START_ADR;		//#4
			wHlsZeroTestAdrEnd_1 = SP_XXXX_SW2_START_ADR+0;		//#8
		}
		wHlsZeroTestAdrStart_2 = ADR_DI_48;
		wHlsZeroTestAdrEnd_2 = ADR_DI_58;
	}

	//前回検査でエラーのとき検査開始時に出力ONのままになるときがあるので、出力値を全クリア
	SnOutputInit(FALSE);
	//Sleep_Cnt(50);
	
	//bit walk test
	for( ii=0; ii<iotbl[wPatternNo].wTestBitNum; ii++ ){
	//for( wOutBit=0; wOutBit<=15; wOutBit++ ){
		wOutVal = 0x0001 << iotbl[wPatternNo].wOutBit[ii];
		wChkVal = 0x0001 << iotbl[wPatternNo].wInBit[ii];
		DATAout(BoardId, iotbl[wPatternNo].wOutAdr, wOutVal);
		Sleep_Cnt(50);

		//入力値が正しく取得できることを確認する
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, iotbl[wPatternNo].wInAdr, (LPINT*)&dwIndata);
			dwIndata = dwIndata & (DWORD)wInMaskBit;	//アナログデータビットは0でマスクする
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//データ不一致エラー

				//出力側MILコネクタ番号を文字列で取得
				GetSpInternalSignalName(iotbl[wPatternNo].wOutAdr, iotbl[wPatternNo].wOutBit[ii], "Out");
				strcpy(&cSignalNameWork[0][0], m_cSpInternalSignalName);

				//出力側信号名を取得
				GetPinNumMilTerm2(iotbl[wPatternNo].wOutAdr, iotbl[wPatternNo].wOutBit[ii]);
				strcpy(&cPinNameWork[0][0], m_cMilConnectorPinName);
				
				//入力側MILコネクタ番号を文字列で取得
				GetSpInternalSignalName(iotbl[wPatternNo].wInAdr, iotbl[wPatternNo].wInBit[ii], "In");
				strcpy(&cSignalNameWork[1][0], m_cSpInternalSignalName);

				//入力側信号名を取得
				GetPinNumMilTerm2(iotbl[wPatternNo].wInAdr, iotbl[wPatternNo].wInBit[ii]);
				strcpy(&cPinNameWork[1][0], m_cMilConnectorPinName);

				LogPrintfCsv("SPxxxx_DioLog.csv",
					"[TestPtn%d] Out_Adr%d%s=0x%04X(Bit%d%s)→In_Adr%d%s=0x%04X(Bit%d%s) [(WalkingテストNG:)データ不一致]\n", 
						wPatternNo,
						iotbl[wPatternNo].wOutAdr,
						&cSignalNameWork[0][0],			//出力側MILコネクタ番号
						wOutVal,
						iotbl[wPatternNo].wOutBit[ii],	//出力側信号名
						&cPinNameWork[0][0],

						iotbl[wPatternNo].wInAdr,
						&cSignalNameWork[1][0],			//入力側MILコネクタ番号
						(int)dwIndata,
						iotbl[wPatternNo].wInBit[ii],
						&cPinNameWork[1][0]);			//入力側信号名
			}
			else{
				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wOutAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS通信エラー
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d 通信エラー(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wOutAdr, termerr);
				}

				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wInAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS通信エラー
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d 通信エラー(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wInAdr, termerr);
				}
			}

			if( bRet == TRUE && bZeroChkEnable == TRUE){
				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_1, wHlsZeroTestAdrEnd_1, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zeroチェックエラー（0以外の入力を検出）
				}

				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_2, wHlsZeroTestAdrEnd_2, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zeroチェックエラー（0以外の入力を検出）
				}
			}

			if( bRet != TRUE ){
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);

		if( bRet != TRUE ){
			break;
		}
	}

	//全点ON確認
	if( bRet == TRUE ){
		wOutVal = 0x0000;
		wChkVal = 0x0000;
		for( ii=0; ii<iotbl[wPatternNo].wTestBitNum; ii++ ){
			wOutVal = wOutVal | (0x0001 << iotbl[wPatternNo].wOutBit[ii]);
			wChkVal = wChkVal | (0x0001 << iotbl[wPatternNo].wInBit[ii]);
		}
		DATAout(BoardId, iotbl[wPatternNo].wOutAdr, wOutVal);
		Sleep_Cnt(50);

		//入力値が正しく取得できることを確認する
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, iotbl[wPatternNo].wInAdr, (LPINT*)&dwIndata);
			dwIndata = dwIndata & (DWORD)wInMaskBit;	//アナログデータビットは0でマスクする
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//データ不一致エラー
				LogPrintfCsv("SPxxxx_DioLog.csv",
					"[TestPtn%d]  Out_Adr%d=0x%04X→In_Adr%d=0x%04X [(全ONテストNG:)データ不一致]\n", 
						wPatternNo,
						iotbl[wPatternNo].wOutAdr,
						wOutVal,
						iotbl[wPatternNo].wInAdr,
						(int)dwIndata);
			}
			else{
				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wOutAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS通信エラー
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d 通信エラー(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wOutAdr, termerr);
				}

				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wInAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS通信エラー
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d 通信エラー(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wInAdr, termerr);
				}
			}

			if( bRet == TRUE && bZeroChkEnable == TRUE){
				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_1, wHlsZeroTestAdrEnd_1, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zeroチェックエラー（0以外の入力を検出）
				}

				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_2, wHlsZeroTestAdrEnd_2, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zeroチェックエラー（0以外の入力を検出）
				}
			}
			if( bRet != TRUE ){
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);
	}

	//全点OFF確認
	if( bRet == TRUE ){
		wOutVal = 0x0000;
		wChkVal = 0x0000;
		DATAout(BoardId, iotbl[wPatternNo].wOutAdr, wOutVal);
		Sleep_Cnt(50);

		//入力値が正しく取得できることを確認する
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, iotbl[wPatternNo].wInAdr, (LPINT*)&dwIndata);
			dwIndata = dwIndata & (DWORD)wInMaskBit;	//アナログデータビットは0でマスクする
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//データ不一致エラー
				LogPrintfCsv("SPxxxx_DioLog.csv",
					"[TestPtn%d]  Out_Adr%d=0x%04X→In_Adr%d=0x%04X [(全OFFテストNG:)データ不一致]\n", 
						wPatternNo,
						iotbl[wPatternNo].wOutAdr,
						wOutVal,
						iotbl[wPatternNo].wInAdr,
						(int)dwIndata);
			}
			else{
				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wOutAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS通信エラー
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d 通信エラー(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wOutAdr, termerr);
				}

				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wInAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS通信エラー
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d 通信エラー(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wInAdr, termerr);
				}
			}
			if( bRet == TRUE && bZeroChkEnable == TRUE){
				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_1, wHlsZeroTestAdrEnd_1, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zeroチェックエラー（0以外の入力を検出）
				}

				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_2, wHlsZeroTestAdrEnd_2, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zeroチェックエラー（0以外の入力を検出）
				}
			}
			if( bRet != TRUE ){
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);
	}

	if( bRet == TRUE ){
		LogPrintfCsv("SPxxxx_DioLog.csv", "DIO TEST Pattern%d <OK>\n", wPatternNo);
	}
	else{
		LogPrintfCsv("SPxxxx_DioLog.csv", "DIO TEST Pattern%d <NG>\n", wPatternNo);
	}
	return bRet;
}
///////////////////////////////////////////////
// DIO折り返しテスト (1bitのみ)
#define LPBACK_READ_NUM		32		//読み出し繰り返し回数
BOOL DioLoopBackTest( WORD wOutAdr, WORD wOutBit, WORD wInAdr, WORD wInBit )
{
	DWORD	dwInData;
	DWORD	dwOutData;
	int		ii;
	BOOL	berr = TRUE;
	int		nInputVal;

	if( BoardId != 0xffff ){
		//折り返しOFF入力を確認する
		dwOutData = (DWORD)(0x0001 << wInBit);
		DATAout(BoardId, wOutAdr, (WORD)dwOutData);
		Sleep_Cnt(100);
		printf("\n");

		//折り返し入力値がONになるまでWAITする(SW操作待ち)
		while(ForeverLoop){
			Sleep_Cnt(10);
			if( _kbhit() ){
				nInputVal = _getch();
				if( nInputVal == 0x1B ){
					berr = FALSE;
					printf("\a折り返し入力テストがキャンセルされました\a\n");
					break;
				}
			}

			dwInData = dwInData & (0x0001 << wInBit);
			DATAin(BoardId, wInAdr, (LPINT*)&dwInData);
			if( dwInData == dwOutData ){	//データの折り返し入力OK
				//printf("ON折り返し入力OK                 \n");
				break;
			}
			else{
				printf("\r折り返し入力待ち... (Esc:中止)\r");
			}
		}
		
		//安定して入力値が読み出せることを確認する		
		for( ii=0; ii<LPBACK_READ_NUM; ii++ ){	//32回
			Sleep_Cnt(5);
			DATAin(BoardId, wInAdr, (LPINT*)&dwInData);
			dwInData = dwInData & (0x0001 << wInBit);
			if( dwInData != dwOutData ){
				berr = FALSE;
			}
		}

		//折り返しOFF入力を確認する
		if( berr == TRUE ){
			dwOutData = 0;
			DATAout(BoardId, wOutAdr, (WORD)dwOutData);
			Sleep_Cnt(100);
			for( ii=0; ii<LPBACK_READ_NUM; ii++ ){	//32回
				Sleep_Cnt(5);
				DATAin(BoardId, wInAdr, (LPINT*)&dwInData);
				dwInData = dwInData & (0x0001 << wInBit);
				if( dwInData != dwOutData ){
					berr = FALSE;
				}
			}

			printf("[%s]Adr%d-B%d → Adr%d-A%d折り返し入力          \n",
					((berr == TRUE) ? "OK" : "NG"),
					wOutAdr, wOutBit, wInAdr, wInBit);
		}
	}
	else{
		berr = FALSE;
	}
	return berr;
}

///////////////////////////////////////////////////////
//D/A出力精度確認(SP_XXXX_BOARD用)
#define	DA_OUTPUT_RANGE_CHK_NUM		(3)
WORD	m_wDaOutputStdVal[DA_OUTPUT_RANGE_CHK_NUM]=
//MIN,    MID,     MAX
{ 0x000,  0x2000,  0x3FFF};

double	m_fDaOutputStdVoltage[DA_OUTPUT_RANGE_CHK_NUM]=
//MIN,   NearMin,  MID, NearMax,  MAX
{ 0.00,   2.50,   5.00};

/***
#define	DA_OUTPUT_RANGE_CHK_NUM		(5)
WORD	m_wDaOutputStdVal[DA_OUTPUT_RANGE_CHK_NUM]=
//MIN,   NearMin,   MID,   NearMax,  MAX
{ 0x000,   0x021,  0x2000,  0x3FDE,  0x3FFF};

double	m_fDaOutputStdVoltage[DA_OUTPUT_RANGE_CHK_NUM]=
//MIN,   NearMin,  MID, NearMax,  MAX
{ 0.00,   0.01,   2.50,   4.99,   5.00};
***/
BOOL SPxxxxDaoutChk()
{
	BOOL	bRetCont=TRUE;
	DWORD	err;
	int		nExitFlg = 0;
	int		nInputVal;
	int		nChNum=0;
	//int		nIval=0;
	int		ii, jj;
	//int		jj, kk;
	//int		nResult[4][3][2];
	//int		nClrResult[4];
	//WORD	wDoVal = 0;
	WORD	wTermAdr;
	BOOL	bTestResult;

	double	fDADatDim[DA_INPUT_DATA_NUM];
	double	fDADatAve;
	double	fDADatMin;
	double	fDADatMax;
	double	fVoltageVal;
	double	fCalcVal;

	HANDLE	forMultiMaterHandles = NULL;
	BOOL	retStart_HP34401A=0;
	BOOL	retChangeVA_DC_HP34401A;

	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv("SPxxxx_DaLog.csv", 
		"■■■ [%s] D/A出力精度確認 %s %s START ■■■\n", m_cSPxxxxBoardName, strDate, strTime);

	////SP_XXXX と同一ポートを使うので、接続を一旦CLOSEする
	//if( m_hSpComHandle != INVALID_HANDLE_VALUE ){
	//	CloseHandle(m_hSpComHandle);
	//	m_hSpComHandle = INVALID_HANDLE_VALUE;
	//}

	for( nChNum=0; nChNum<SP_XXXX_DACH_MAX; nChNum++ ){
		printf("\n------------------------------------------------\n");
		printf("　マルチメータのプローブをDA ch%dに接続してください。\n", nChNum);
		printf("　RS-232C接続をマルチメータに変更してください。\n");
		printf("\n------------------------------------------------\n");
		printf("　SAVENET(6M/H)通信接続が行われていることを確認してください。\n");
		printf("--------------------------------------------------\n");
		printf("準備ができたら何かキーを押してください。\n\n");
		nInputVal = KeyInputWait();
		if ( nInputVal == 0x1B ){
			nExitFlg = 1;
			bRetCont = FALSE;
			break;
		}

		if( nChNum == 0 ){
			//マルチメータで計測するための準備
			retStart_HP34401A = Start_HP34401A(
				m_cComPort,
				&forMultiMaterHandles	//通信リソースのハンドルを返すアドレス
			);
			if( ! retStart_HP34401A ){
				LogPrintf("MultiMeter 34401A Initialize Error!!\n");
				nExitFlg = 1;
				bRetCont = FALSE;
				break;
			}

			//マルチメータのレンジを電圧入力に設定する
			retChangeVA_DC_HP34401A = ChangeVoltageDC_HP34401A(
				forMultiMaterHandles	//通信リソースハンドル
			);
			if( ! retChangeVA_DC_HP34401A ){
				LogPrintf("MultiMeter 34401A VoltageSetting Error!!\n");
				End_HP34401A(forMultiMaterHandles);	//通信リソースのハンドルを閉じる
				nExitFlg = 1;
				bRetCont = FALSE;
				break;
			}
		}

		wTermAdr = (unsigned short)(nChNum+(SP_XXXX_SW2_START_ADR+2));
		//
		//[DA] 0.00V(0x000), 0.01V(0x021), 2.50V(0x2000), 4.99V(0x3FDE), 5.00V(0x3FFF) をマルチメータでサンプリングする
		//
		for( ii=0; ii<DA_OUTPUT_RANGE_CHK_NUM; ii++ ){
			//SAVENETからDA出力値を設定する
			err = DATAout(BoardId, wTermAdr, m_wDaOutputStdVal[ii]);
			if( err != SNMA_NO_ERROR ){
				printf("DATAout() err=%ld\a\n", err);
				HitAnyKeyWait();
				nExitFlg = 1;
				bRetCont = FALSE;
				break;
			}
			Sleep_Cnt(1000);
			
			//マルチメータからDA出力値をサンプリングする
			for( jj=0; jj<DA_INPUT_DATA_NUM; jj++ ){
				if( _kbhit() ){
					nInputVal = _getch();
					if( nInputVal == 0x1B ){
						nExitFlg = 1;
						bRetCont = FALSE;
						break;
					}
				}
				Sleep_Cnt(10);
				if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
					nExitFlg = 1;
					bRetCont = FALSE;
					break;
				}
				else{
					printf("(%02d/%02d) ch%d:%fV\n", jj+1, DA_INPUT_DATA_NUM, nChNum, fVoltageVal);
				}
				fDADatDim[jj] = fVoltageVal;
			}

			//結果出力
			if( bRetCont == TRUE && nExitFlg == 0 ){
				bTestResult = TRUE;

				GetDaMinMaxAve(fDADatDim, &fDADatMin, &fDADatMax, &fDADatAve);

				printf("\n");
				LogPrintfCsv("SPxxxx_DaLog.csv", "----<(D/A ch%d) %5.3fV 精度確認>---\n", nChNum, m_fDaOutputStdVoltage[ii]);
				LogPrintfCsv("SPxxxx_DaLog.csv", "CheckType, Voltafge(V), Precision(%%), Result\n");

				//MIN
				fCalcVal = (double)(fabs((double)m_fDaOutputStdVoltage[ii] - fDADatMin) * 100.0) / (double)(5.0);
				LogPrintfCsv("SPxxxx_DaLog.csv", "MIN, %f, %f, %s\n",
					fDADatMin, fCalcVal, (fCalcVal > DA_OK_RATE) ? "NG": "OK");
				printf("(ch%d)%5.3fV DA精度(MIN)=%5.3f%%\n\n", nChNum, m_fDaOutputStdVoltage[ii], fCalcVal);
				if( fCalcVal > DA_OK_RATE ){
				//	nResult[nChNum-1][jj][kk] = FALSE;
					bTestResult = FALSE;
				}

				//MAX
				fCalcVal = (double)(fabs((double)m_fDaOutputStdVoltage[ii] - fDADatMax) * 100.0) / (double)(5.0);
				LogPrintfCsv("SPxxxx_DaLog.csv", "MAX, %f, %f, %s\n",
					fDADatMax, fCalcVal, (fCalcVal > DA_OK_RATE) ? "NG": "OK");
				printf("(ch%d)%5.3fV DA精度(MAX)=%5.3f%%\n\n", nChNum, m_fDaOutputStdVoltage[ii], fCalcVal);
				if( fCalcVal > DA_OK_RATE ){
				//	nResult[nChNum-1][jj][kk] = FALSE;
					bTestResult = FALSE;
				}

				//AVE
				fCalcVal = (double)(fabs((double)m_fDaOutputStdVoltage[ii] - fDADatAve) * 100.0) / (double)(5.0);
				LogPrintfCsv("SPxxxx_DaLog.csv", "AVE, %f, %f, %s\n",
					fDADatAve, fCalcVal, (fCalcVal > DA_OK_RATE) ? "NG": "OK");
				printf("(ch%d)%5.3fV DA精度(AVE)=%5.3f%%\n\n", nChNum, m_fDaOutputStdVoltage[ii], fCalcVal);
				if( fCalcVal > DA_OK_RATE ){
				//	nResult[nChNum-1][jj][kk] = FALSE;
					bTestResult = FALSE;
				}

				if( bTestResult != TRUE ){
					printf("\n検査精度が%5.2f%%を超えました\a\a\n", DA_OK_RATE);
					printf("\n検査を中止しますか？(y or n) -->");
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						nExitFlg = 1;
						bRetCont = FALSE;
						break;
					}
				}
			}
			else{
				break;
			}
		}
	
		if( bRetCont == FALSE ){
			break;
		}
	}

	//マルチメータ通信リソースのハンドルを閉じる
	if( forMultiMaterHandles != NULL ){
		End_HP34401A(forMultiMaterHandles);
	}

	////RS232C接続をSP_XXXXに戻す
	//if( m_hSpComHandle == INVALID_HANDLE_VALUE ){
	//	RS_SpComOpen();
	//	printf("\n------------------------------------------------\n");
	//	printf("　RS-232C接続をUPPER BOARD側に戻してください。\n");
	//	printf("　準備ができたら何かキーを押してください。\n\n");
	//	printf("\n------------------------------------------------\n");
	//	KeyInputWait();
	//}

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv("SPxxxx_DaLog.csv", 
		"■■■ [%s] D/A出力精度確認 %s %s FINISH ■■■\n", m_cSPxxxxBoardName, strDate, strTime);

	return bRetCont;
}

BOOL SPxxxxDiChk()
{
	int		nInputVal = -1;
	int		nAdAdrOffs=0;
	DWORD	dwInData[8];
	DWORD	dwInData_old[8];
	BOOL	bRetCont=TRUE;
	BOOL	bFirstDisp=TRUE;

	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv("SPxxxx_DiLog.csv", 
		"■■■ [%s] DI(SIG)入力確認 %s %s START ■■■\n", m_cSPxxxxBoardName, strDate, strTime);

	memset(dwInData, 0, sizeof(dwInData));
	memset(dwInData_old, 0, sizeof(dwInData_old));
	printf("--- Analog Connector DI入力確認 ---\n");
	switch(m_nSPxxxxBoardID){
	case SP_BID_UPPER_IF_BOARD:
		nAdAdrOffs = 7;		//Adr7,10,11,12
		printf("[CN51(SIG1,SIG2)], [CN61～CN63(DIN)]\n");
		break;
	case SP_BID_FRONT_IF_BOARD:
		nAdAdrOffs = 7;		//Adr7,8,9,10
		printf("[CN51～CN52(SIG1)], [CN53～CN54(SIG1,SIG2)]\n");
		break;
	}
	printf("Key Input (OK='Y', NG='N', 中止='Esc')\n\n");
	while(ForeverLoop){
		switch(m_nSPxxxxBoardID){
		case SP_BID_UPPER_IF_BOARD:
			DATAin(BoardId, (unsigned short)(nAdAdrOffs+0), (LPINT*)&dwInData[0]);
			DATAin(BoardId, (unsigned short)(nAdAdrOffs+3), (LPINT*)&dwInData[1]);
			DATAin(BoardId, (unsigned short)(nAdAdrOffs+4), (LPINT*)&dwInData[2]);
			DATAin(BoardId, (unsigned short)(nAdAdrOffs+5), (LPINT*)&dwInData[3]);
			break;
		case SP_BID_FRONT_IF_BOARD:
			DATAin(BoardId, (unsigned short)(nAdAdrOffs+0), (LPINT*)&dwInData[0]);
			DATAin(BoardId, (unsigned short)(nAdAdrOffs+1), (LPINT*)&dwInData[1]);
			DATAin(BoardId, (unsigned short)(nAdAdrOffs+2), (LPINT*)&dwInData[2]);
			DATAin(BoardId, (unsigned short)(nAdAdrOffs+3), (LPINT*)&dwInData[3]);
			break;
		}

		if( (bFirstDisp == TRUE) ||
			(dwInData[0] & 0xC000) != (dwInData_old[0] & 0xC000) ||
			(dwInData[1] & 0xC000) != (dwInData_old[1] & 0xC000) ||
			(dwInData[2] & 0xC000) != (dwInData_old[2] & 0xC000) ||
			(dwInData[3] & 0xC000) != (dwInData_old[3] & 0xC000) ){
			bFirstDisp = FALSE;
			memcpy(dwInData_old, dwInData, sizeof(dwInData));

			switch(m_nSPxxxxBoardID){
			case SP_BID_UPPER_IF_BOARD:
				printf("[CN51=%d,%d], [CN61=%d], [CN62=%d], [CN63=%d]\r",
						((dwInData[0] & 0x4000) ? 1:0), ((dwInData[0] & 0x8000) ? 1:0),
						((dwInData[1] & 0x4000) ? 1:0),
						((dwInData[2] & 0x4000) ? 1:0),
						((dwInData[3] & 0x4000) ? 1:0));
				break;
			case SP_BID_FRONT_IF_BOARD:
				printf("[CN51=%d], [CN52=%d], [CN53=%d,%d], [CN54=%d,%d]\r",
						((dwInData[0] & 0x4000) ? 1:0),
						((dwInData[1] & 0x4000) ? 1:0),
						((dwInData[2] & 0x4000) ? 1:0), ((dwInData[2] & 0x8000) ? 1:0),
						((dwInData[3] & 0x4000) ? 1:0), ((dwInData[3] & 0x8000) ? 1:0));
				break;
			}
		}


		if(_kbhit()){
			printf("\n");
			nInputVal = _getch();
			if( nInputVal == 0x1B ){
				printf("中断しました。\a\n");
				LogPrintfCsv("SPxxxx_DiLog.csv", " DI(SIG)入力確認 (中断)\n");
				break;
			}
			else if( nInputVal == 'y' || nInputVal == 'Y' || nInputVal == 0x0d ){
				LogPrintfCsv("SPxxxx_DiLog.csv", " DI(SIG)入力確認 (OK)\n");
				break;
			}
			else if( nInputVal == 'n' || nInputVal == 'N' ){
				LogPrintfCsv("SPxxxx_DiLog.csv", " DI(SIG)入力確認 (NG)\n");
				break;
			}
			printf("\a");
		}
		Sleep_Cnt(50);
	}

	printf("\n\n");
	/////////////////////////////////////////////////////////////////////
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv("SPxxxx_DiLog.csv", 
		"■■■ [%s] DI(SIG)入力確認 %s %s FINISH ■■■\n", m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;

}

///////////////////////////////////////////////////////
//SP_xxxx_BOARDのEEPROMに設定されているボードIDを取得（または変更）する
int RS_SpUpdateBoardID(int nNewBoardID, int *npFirmVer)
{
	int	ii;
	int	nSendSize=0;
	int	nRecvSize=0;
	int	nRecvBID=-1;
	char cSaveEnable;
	char cTemp[8];
	char *endptr;
	unsigned char	ucCheckSumCalc;
	unsigned char	ucCheckSumRecv;
	memset(m_cSendPacket, 0, sizeof(m_cSendPacket));
	memset(m_cRecvPacket, 0, sizeof(m_cRecvPacket));

	m_cSendPacket[nSendSize++] = COM_STX;
	m_cSendPacket[nSendSize++] = '0';

	*npFirmVer= 0x9999;

	//電文長
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//テストモード
	m_cSendPacket[nSendSize++] = SP_RS_MODE_EEPSAVE;

	//出力データ有効・無効フラグ
	if( nNewBoardID == SP_BID_CHECK_ONLY ){
		cSaveEnable = '0';		//チェックのみ時は無効フラグをセットする
	}
	else{
		cSaveEnable = '1';		//ボードID変更時は有効フラグをセットする
	}
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		m_cSendPacket[nSendSize++] = cSaveEnable;	//ボードID変更時はBit15の状態に関係無い
	}

	//出力データ
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		sprintf((char*)&m_cSendPacket[nSendSize], "%04X", nNewBoardID);
		nSendSize+=4;
	}

	//チェックサム計算
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARDに対してパケット送信、及びデータ受信待ち
	nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
	if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD ){
		ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
		ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
		if( ucCheckSumCalc == ucCheckSumRecv ){
			memset(cTemp, 0, sizeof(cTemp));
			memcpy(cTemp, &m_cRecvPacket[12], 4);
			nRecvBID = (int)strtol(cTemp, &endptr, 16);

			memcpy(cTemp, &m_cRecvPacket[40], 4);
			*npFirmVer = (int)strtol(cTemp, &endptr, 16);
		}
		else{
			printf("受信データチェックサムエラー(Hit any key)\a\n");
			KeyInputWait();
		}
	}
#ifndef	_RS_DEBUG_MODE
	else{
		printf("受信データサイズ(=%d)エラー(Hit any key)\a\n", nRecvSize);
		KeyInputWait();
	}
#else
	nRecvBID = SP_BID_FRONT_IF_BOARD;
#endif
	return nRecvBID;
}

///////////////////////////////////////////////////////
//アナログ入力生データを取得する
int RS_SpInputAdRawVal(int nch)
{
	int	ii;
	int	nSendSize=0;
	int	nRecvSize=0;
	char *endptr;
	unsigned char	ucCheckSumCalc;
	unsigned char	ucCheckSumRecv;
	char cTemp[8];
	int	nAdRawVal=0xffff;

	memset(m_cSendPacket, 0, sizeof(m_cSendPacket));
	memset(m_cRecvPacket, 0, sizeof(m_cRecvPacket));

	m_cSendPacket[nSendSize++] = COM_STX;
	m_cSendPacket[nSendSize++] = '0';

	//電文長
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//テストモード
	m_cSendPacket[nSendSize++] = SP_RS_MODE_RAW;

	//出力データ有効・無効フラグ
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		m_cSendPacket[nSendSize++] = '0';	//全て無効
	}

	//出力データ
	for(ii=0; ii<(SP_XXXX_ADCH_MAX*4); ii++){
		m_cSendPacket[nSendSize++] = '0';
	}

	//チェックサム計算
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARDに対してパケット送信、及びデータ受信待ち
	nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
	if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD ){
		ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
		ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
		if( ucCheckSumCalc == ucCheckSumRecv ){
			if( m_cRecvPacket[6+nch] == '1' ){	//入力値は有効？
				memset(cTemp, 0, sizeof(cTemp));
				memcpy(cTemp, &m_cRecvPacket[12+4*nch], 4);
				nAdRawVal = (int)strtol(cTemp, &endptr, 16);
			}
			else{
				printf("生データ入力不可(Hit any key)\a\n");
				KeyInputWait();
			}
		}
		else{
			printf("受信データチェックサムエラー(Hit any key)\a\n");
			KeyInputWait();
		}
	}
#ifndef	_RS_DEBUG_MODE
	else{
		printf("受信データサイズ(=%d)エラー(Hit any key)\a\n", nRecvSize);
		KeyInputWait();
	}
#else
	nAdRawVal = 0x1000;
#endif
	return nAdRawVal;
}

////////////////////////////////////////
// アナログ入力補正後の値を更新する
int RS_SpInputAdCorrectVal(int nch)
{
	int	ii;
	int	nSendSize=0;
	int	nRecvSize=0;
	char *endptr;
	unsigned char	ucCheckSumCalc;
	unsigned char	ucCheckSumRecv;
	char cTemp[8];
	int	nAdCorrectVal = 0xffff;

	memset(m_cSendPacket, 0, sizeof(m_cSendPacket));
	memset(m_cRecvPacket, 0, sizeof(m_cRecvPacket));

	m_cSendPacket[nSendSize++] = COM_STX;
	m_cSendPacket[nSendSize++] = '0';

	//電文長
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//テストモード
	m_cSendPacket[nSendSize++] = SP_RS_MODE_NORMAL;

	//出力データ有効・無効フラグ
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		m_cSendPacket[nSendSize++] = '0';	//全て無効
	}

	//出力データ
	for(ii=0; ii<(SP_XXXX_ADCH_MAX*4); ii++){
		m_cSendPacket[nSendSize++] = '0';
	}

	//チェックサム計算
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARDに対してパケット送信、及びデータ受信待ち
	nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
	if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD  ){
		ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
		ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
		if( ucCheckSumCalc == ucCheckSumRecv ){
			if( m_cRecvPacket[6+nch] == '1' ){	//入力値は有効？
				memset(cTemp, 0, sizeof(cTemp));
				memcpy(cTemp, &m_cRecvPacket[12+4*nch], 4);
				nAdCorrectVal = (int)strtol(cTemp, &endptr, 16);
			}
			else{
				printf("AD補正後のデータ入力不可(Hit any key)\a\n");
				KeyInputWait();
			}
		}
		else{
			printf("受信データチェックサムエラー(Hit any key)\a\n");
			KeyInputWait();
		}
	}
#ifndef	_RS_DEBUG_MODE
	else{
		printf("受信データサイズ(=%d)エラー(Hit any key)\a\n", nRecvSize);
		KeyInputWait();
	}
#else
	nAdCorrectVal = 0x1000;
#endif
	return nAdCorrectVal;
}

////////////////////////////////////////
// アナログ入力補正値を更新する
BOOL RS_SpSetAdOffsetGain(DWORD *dwAdOffset, DWORD *dwAdGain, DWORD *dwReadOffset, DWORD *dwReadGain)
{
	BOOL berr = TRUE;
	int	ii, jj;
	int	nSendSize=0;
	int	nRecvSize=0;
	char *endptr;
	unsigned char	ucCheckSumCalc;
	unsigned char	ucCheckSumRecv;
	DWORD	dwRecvOffset[SP_XXXX_ADCH_MAX];
	DWORD	dwRecvGain[SP_XXXX_ADCH_MAX];
	DWORD	dwUpdateFlg[SP_XXXX_ADCH_MAX];
	char cTemp[8];

	for( jj=0; jj<2; jj++ ){	//jj 0:Offset, 1:Gain
		memset(m_cSendPacket, 0, sizeof(m_cSendPacket));
		memset(m_cRecvPacket, 0, sizeof(m_cRecvPacket));
		memset(dwUpdateFlg, 0, sizeof(dwUpdateFlg));

		m_cSendPacket[nSendSize++] = COM_STX;
		m_cSendPacket[nSendSize++] = '0';

		//電文長
		sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
		nSendSize +=3;

		//テストモード
		m_cSendPacket[nSendSize++] = SP_RS_MODE_ADADJ;

		//出力データ有効・無効フラグ
		for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
			if( dwAdOffset[ii] == 0xffff || dwAdGain[ii] == 0xffff ){
				m_cSendPacket[nSendSize++] = '0';		//更新なし
			}
			else{
				m_cSendPacket[nSendSize++] = '1';		//更新あり
				dwUpdateFlg[ii] = 0x8000;	//更新あり
			}
		}

		//出力データ
		for( ii=0; ii<SP_XXXX_ADCH_MAX; ii++ ){
			if( jj == 0 ){	//Offset
				sprintf((char*)&m_cSendPacket[nSendSize], "%04X", ((dwAdOffset[ii] & 0x0fff) | dwUpdateFlg[ii]));
				nSendSize+=4;
			}
			else{			//Gain
				sprintf((char*)&m_cSendPacket[nSendSize], "%04X", ((dwAdGain[ii] & 0x0fff) | dwUpdateFlg[ii] | 0x1000));
				nSendSize+=4;
			}
		}

		//チェックサム計算
		ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
		sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
		nSendSize+=2;

		m_cSendPacket[nSendSize++] = COM_ETX;

		//SP_xxxx_BOARDに対してパケット送信、及びデータ受信待ち
		nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
		if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD  ){
			ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
			ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
			if( ucCheckSumCalc == ucCheckSumRecv ){
				for( ii=0; ii<SP_XXXX_ADCH_MAX; ii++ ){
					memset(cTemp, 0, sizeof(cTemp));
					memcpy(cTemp, &m_cRecvPacket[12+ii*4], 4);
					if( jj == 0 ){	//Offset
						dwRecvOffset[ii] = (DWORD)strtol(cTemp, &endptr, 16);
						if( (dwAdOffset[ii] != 0xffff) && (dwAdGain[ii] != 0xffff) ){
							if( (dwAdOffset[ii] & 0x0fff) != (dwRecvOffset[ii] & 0x0fff) ){
								printf("(ch%d AD)OFFSET 送信(0x%X),受信(0x%X)データ不一致\a\n", ii, dwAdOffset[ii], dwRecvOffset[ii]);
								berr = FALSE;	//送信データと受信データが不一致
							}
						}
						if( dwReadOffset != NULL ){
							dwReadOffset[ii] = dwRecvOffset[ii] & 0x0fff;
						}
					}
					else{			//Gain
						dwRecvGain[ii] = (DWORD)strtol(cTemp, &endptr, 16);
						if( (dwAdOffset[ii] != 0xffff) && (dwAdGain[ii] != 0xffff) ){
							if( (dwAdGain[ii] & 0x0fff) != (dwRecvGain[ii] & 0x0fff) ){
								printf("(ch%d AD)GAIN 送信(0x%X),受信(0x%X)データ不一致\a\n", ii, dwAdGain[ii], dwRecvGain[ii]);
								berr = FALSE;	//送信データと受信データが不一致
							}
						}
						if( dwReadGain != NULL ){
							dwReadGain[ii] = dwRecvGain[ii] & 0x0fff;
						}
					}
				}
			}
			else{
				printf("受信データチェックサムエラー(Hit any key)\a\n");
				KeyInputWait();
				berr = FALSE;
			}
		}
#ifndef	_RS_DEBUG_MODE
		else{
			printf("受信データサイズ(=%d)エラー(Hit any key)\a\n", nRecvSize);
			KeyInputWait();
			berr = FALSE;
		}
#endif
	}
	return berr;
}

////////////////////////////////////////
// アナログ出力補正値を更新する
BOOL RS_SpSetDaOffsetGain(DWORD *dwDaOffset, DWORD *dwDaGain, DWORD *dwReadOffset, DWORD *dwReadGain)
{
	BOOL berr = TRUE;
	int	ii, jj;
	int	nSendSize=0;
	int	nRecvSize=0;
	char *endptr;
	unsigned char	ucCheckSumCalc;
	unsigned char	ucCheckSumRecv;
	DWORD	dwRecvOffset[SP_XXXX_ADCH_MAX];
	DWORD	dwRecvGain[SP_XXXX_ADCH_MAX];
	DWORD	dwUpdateFlg[SP_XXXX_ADCH_MAX];
	char cTemp[8];

	for( jj=0; jj<2; jj++ ){	//jj 0:Offset, 1:Gain
		memset(m_cSendPacket, 0, sizeof(m_cSendPacket));
		memset(m_cRecvPacket, 0, sizeof(m_cRecvPacket));
		memset(dwUpdateFlg, 0, sizeof(dwUpdateFlg));

		m_cSendPacket[nSendSize++] = COM_STX;
		m_cSendPacket[nSendSize++] = '0';

		//電文長
		sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
		nSendSize +=3;

		//DAテストモード
		m_cSendPacket[nSendSize++] = SP_RS_MODE_DAADJ;

		//出力データ有効・無効フラグ
		for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
			if( ii >= SP_XXXX_DACH_MAX ){
				m_cSendPacket[nSendSize++] = '0';		//更新なし
			}
			else{
				if( dwDaOffset[ii] == 0xffff || dwDaGain[ii] == 0xffff ){
					m_cSendPacket[nSendSize++] = '0';		//更新なし
				}
				else{
					m_cSendPacket[nSendSize++] = '1';		//更新あり
					dwUpdateFlg[ii] = 0x8000;	//更新あり
				}
			}
		}

		//出力データ
		for( ii=0; ii<SP_XXXX_ADCH_MAX; ii++ ){
			if( ii < SP_XXXX_DACH_MAX ){
				if( jj == 0 ){	//Offset
					sprintf((char*)&m_cSendPacket[nSendSize], "%04X", ((dwDaOffset[ii] & 0x0fff) | dwUpdateFlg[ii]));
					nSendSize+=4;
				}
				else{			//Gain
					sprintf((char*)&m_cSendPacket[nSendSize], "%04X", ((dwDaGain[ii] & 0x0fff) | dwUpdateFlg[ii] | 0x1000));
					nSendSize+=4;
				}
			}
			else{
				sprintf((char*)&m_cSendPacket[nSendSize], "%04X", 0x0000);
				nSendSize+=4;
			}
		}

		//チェックサム計算
		ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
		sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
		nSendSize+=2;

		m_cSendPacket[nSendSize++] = COM_ETX;

		//SP_xxxx_BOARDに対してパケット送信、及びデータ受信待ち
		nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
		if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD  ){
			ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
			ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
			if( ucCheckSumCalc == ucCheckSumRecv ){
				for( ii=0; ii<SP_XXXX_DACH_MAX; ii++ ){
					memset(cTemp, 0, sizeof(cTemp));
					memcpy(cTemp, &m_cRecvPacket[12+ii*4], 4);
					if( jj == 0 ){	//Offset
						dwRecvOffset[ii] = (DWORD)strtol(cTemp, &endptr, 16);
						if( (dwDaOffset[ii] != 0xffff) && (dwDaGain[ii] != 0xffff) ){
							if( (dwDaOffset[ii] & 0x0fff) != (dwRecvOffset[ii] & 0x0fff) ){
								printf("(ch%d DA)OFFSET 送信(0x%X),受信(0x%X)データ不一致\a\n", ii, dwDaOffset[ii], dwRecvOffset[ii]);
								berr = FALSE;	//送信データと受信データが不一致
							}
						}
						if( dwReadOffset != NULL ){
							dwReadOffset[ii] = dwRecvOffset[ii] & 0x0fff;
						}
					}
					else{			//Gain
						dwRecvGain[ii] = (DWORD)strtol(cTemp, &endptr, 16);
						if( (dwDaOffset[ii] != 0xffff) && (dwDaGain[ii] != 0xffff) ){
							if( (dwDaGain[ii] & 0x0fff) != (dwRecvGain[ii] & 0x0fff) ){
								printf("(ch%d DA)GAIN 送信(0x%X),受信(0x%X)データ不一致\a\n", ii, dwDaGain[ii], dwRecvGain[ii]);
								berr = FALSE;	//送信データと受信データが不一致
							}
						}
						if( dwReadGain != NULL ){
							dwReadGain[ii] = dwRecvGain[ii] & 0x0fff;
						}
					}
				}
			}
			else{
				printf("受信データチェックサムエラー(Hit any key)\a\n");
				KeyInputWait();
				berr = FALSE;
			}
		}
#ifndef	_RS_DEBUG_MODE
		else{
			printf("受信データサイズ(=%d)エラー(Hit any key)\a\n", nRecvSize);
			KeyInputWait();
			berr = FALSE;
		}
#endif
	}
	return berr;
}


////////////////////////////////////////
// ファームウェアの処理モードを標準モードに戻す
BOOL RS_SpNormalMode()
{
	int	ii;
	BOOL bret = FALSE;
	int	nSendSize=0;
	int	nRecvSize=0;
	char *endptr;
	unsigned char	ucCheckSumCalc;
	unsigned char	ucCheckSumRecv;
	//int	nAdCorrectVal = 0xffff;

	memset(m_cSendPacket, 0, sizeof(m_cSendPacket));
	memset(m_cRecvPacket, 0, sizeof(m_cRecvPacket));

	m_cSendPacket[nSendSize++] = COM_STX;
	m_cSendPacket[nSendSize++] = '0';

	//電文長
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//テストモード
	m_cSendPacket[nSendSize++] = SP_RS_MODE_NORMAL;

	//出力データ有効・無効フラグ
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		m_cSendPacket[nSendSize++] = '0';	//全て無効
	}

	//出力データ
	for(ii=0; ii<(SP_XXXX_ADCH_MAX*4); ii++){
		m_cSendPacket[nSendSize++] = '0';
	}

	//チェックサム計算
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARDに対してパケット送信、及びデータ受信待ち
	nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
	if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD  ){
		ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
		ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
		if( ucCheckSumCalc == ucCheckSumRecv ){
			bret = TRUE;
		}
		else{
			printf("受信データチェックサムエラー(Hit any key)\a\n");
			KeyInputWait();
		}
	}
	return bret;
}

////////////////////////////////////////
// アナログ入出力補正値のEEPROM保存実行
BOOL RS_SpUpdateEeprom(int nChMax)
{
	BOOL	berr=TRUE;
	char	*endptr;
	int		ii;
	int		nSendSize=0;
	int		nRecvSize=0;
	unsigned char	ucCheckSumCalc;
	unsigned char	ucCheckSumRecv;
	memset(m_cSendPacket, 0, sizeof(m_cSendPacket));
	memset(m_cRecvPacket, 0, sizeof(m_cRecvPacket));

	m_cSendPacket[nSendSize++] = COM_STX;
	m_cSendPacket[nSendSize++] = '0';

	//電文長
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//テストモード
	m_cSendPacket[nSendSize++] = SP_RS_MODE_EEPSAVE;

	//出力データ有効・無効フラグ
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		if( ii >= nChMax ){
			m_cSendPacket[nSendSize++] = '0';		//更新なし
		}
		else{
			m_cSendPacket[nSendSize++] = '1';		//更新あり
		}
	}

	//出力データ
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		sprintf((char*)&m_cSendPacket[nSendSize], "%04X", 0x8000);
		nSendSize+=4;
	}

	//チェックサム計算
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARDに対してパケット送信、及びデータ受信待ち
	nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
	if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD ){
		ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
		ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
		if( ucCheckSumCalc != ucCheckSumRecv ){
			printf("受信データチェックサムエラー(Hit any key)\a\n");
			KeyInputWait();
			berr = FALSE;
		}
	}
#ifndef	_RS_DEBUG_MODE
	else{
		printf("受信データサイズ(=%d)エラー(Hit any key)\a\n", nRecvSize);
		KeyInputWait();
		berr = FALSE;
	}
#endif
	return berr;
}

////////////////////////////////////////
// RS-232C送受信データのチェックサムを計算する
unsigned char RS_SpCheckSum(unsigned char *buff, int nsize)
{
	int	ii;
	unsigned int csum=0x00;

	for(ii=0; ii<nsize; ii++ ){
		csum = (unsigned int)(csum + (unsigned int)buff[ii]) & 0x00ff;
	}
	return (unsigned char)csum;
}

////////////////////////////////////////
// 検査基板とのRS-232C通信ポートをオープンする
BOOL RS_SpComOpen()
{
	DCB dcb;					//COMﾎﾟｰﾄDCB構造体
	COMMTIMEOUTS timeouts;		//COMMﾎﾟｰﾄﾀｲﾑｱｳﾄ構造体
	COMMPROP cp;				//COMﾌﾟﾛﾝﾌﾟﾄ構造体
	DWORD dwerr;
	
	if( m_hSpComHandle != INVALID_HANDLE_VALUE ){
		CloseHandle(m_hSpComHandle);
		m_hSpComHandle = INVALID_HANDLE_VALUE;
	}
	
	m_hSpComHandle = CreateFile(m_cSpxxComPort, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if( m_hSpComHandle == INVALID_HANDLE_VALUE ){
		dwerr = GetLastError();
		printf("%s ポートオープンエラー(err=0x%X)\a\n", m_cSpxxComPort, dwerr );
		return FALSE;			//COMMﾎﾟｰﾄｵｰﾌﾟﾝｴﾗｰ
	}

	cp.wPacketLength = sizeof(COMMPROP);					// COMMPROPのｻｲｽﾞ
	GetCommProperties (m_hSpComHandle, &cp);				// COMMﾎﾟｰﾄのﾌﾟﾛﾊﾟﾃｨ
	GetCommState( m_hSpComHandle, &dcb );				// COMﾎﾟｰﾄ条件取得
	dcb.BaudRate = 38400;								// 通信速度
	dcb.ByteSize = 8;									// データ長
	dcb.StopBits = ONESTOPBIT;							// ストップビット
	dcb.fParity = 0;									// パリティチェック 0:無効
	dcb.Parity = NOPARITY;								// パリティなし
	SetCommState( m_hSpComHandle, &dcb );				// COMポート条件設定

	if (!(cp.dwProvCapabilities & PCF_TOTALTIMEOUTS)){
		//このポートはTotalタイムアウトをサポートしていない
		CloseHandle(m_hSpComHandle);
		m_hSpComHandle = INVALID_HANDLE_VALUE;
		printf("%s ポートオープンエラー???\a\n", m_cSpxxComPort);
		return FALSE;
	}

	memset(&timeouts, 0, sizeof(timeouts));
	GetCommTimeouts(m_hSpComHandle, &timeouts);	// ﾀｲﾑｱｳﾄ設定取得
	timeouts.ReadIntervalTimeout = 20;			//2文字間の最大待ち時間
	timeouts.WriteTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 20;	//Writeタイムアウト時間の掛け算係数;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.ReadTotalTimeoutMultiplier = 20;	//Readタイムアウト時間の掛け算係数;
	SetCommTimeouts(m_hSpComHandle, &timeouts);	// ﾀｲﾑｱｳﾄ設定

	return TRUE;
}

//////////////////////////////////////////////
//SP_xxxx_BOARD RS-232Cデータ送受信
int RS_SpSendRecv(unsigned char *cSendPacket, int nSendSize, unsigned char *cRecvPacket, int nRecvSizeMax)
{
	DWORD	nActualWrite;	//実際に書き込んだバイト数
	DWORD	nBytesRead;
	DWORD	dwError;
	COMSTAT	cs;
	BOOL	etx_rcvd=FALSE;
	int		ii;
	int		nRetRecvSize=-1;

	if( m_hSpComHandle == INVALID_HANDLE_VALUE ){
		return nRetRecvSize;	//COMポートオープンエラー
	}
	PurgeComm( m_hSpComHandle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );

	nActualWrite = 0;
	for( ii=0; ii<3; ii++ ){
		//SP_xxxx_BOARDにパケット送信
		WriteFile(m_hSpComHandle, cSendPacket, nSendSize, &nActualWrite, NULL);
		if( (DWORD)nSendSize == nActualWrite ){
			break;
		}
	}

	//SP_xxxx_BOARDからの応答待ち
	if( (DWORD)nSendSize == nActualWrite ){
#ifndef	_RS_DEBUG_MODE
		//STX受信待ち
		for( ii=0; ii<3; ii++ ){
			Sleep_Cnt(50);
			if( ReadFile(m_hSpComHandle, &cRecvPacket[0], 1, &nBytesRead, NULL) == 0 ){
				ClearCommError(m_hSpComHandle, &dwError, &cs);	// 受信エラーの後処理
			}
			else{
				if( nBytesRead == 1 ){
					if( cRecvPacket[0] == COM_STX ){
						nRetRecvSize = 1;
						break;
					}
				}
				else{
					Sleep_Cnt(1000);
				}
			}
		}

		//ETX受信待ち
		if( nRetRecvSize == 1 ){
			for( ii=1; ii<nRecvSizeMax; ii++ ){
				if( ReadFile(m_hSpComHandle, &cRecvPacket[ii], 1, &nBytesRead, NULL) == 0 ){
					ClearCommError(m_hSpComHandle, &dwError, &cs);	// 受信エラーの後処理
					nRetRecvSize = -1;
					break;
				}
				else{
					if( nBytesRead >= 1 ){
						nRetRecvSize++;
						if( cRecvPacket[ii] == COM_ETX ){
							etx_rcvd = TRUE;
							break;
						}
					}
					else{
						nRetRecvSize = -1;
						break;
					}
				}
			}
		}
#endif
	}

	if( etx_rcvd != TRUE ){
		nRetRecvSize = -1;
	}
	return nRetRecvSize;
}

/////////////////////////////////////////////////////
// SN-4016-SRCM, SN-4016-STCM ビット番号→MIL20ピンのピン番号変換
const int SnChangeMilCnPinNum[]={
	20, 18, 16, 14, 12, 10, 8, 6, 19, 17, 15, 13, 11, 9, 7, 5
};
int GetPinNumMilTerm(WORD wBitNum)
{
	if( wBitNum < 16 ){
		return SnChangeMilCnPinNum[wBitNum];
	}
	return 0;
}
char* GetPinNumMilTerm2(WORD wAdr, WORD wBitNum)
{
	if( wAdr > 12 ){	//12:検査品の最終Adr
		sprintf(m_cMilConnectorPinName, ",MIL%dpin", GetPinNumMilTerm(wBitNum));
	}
	else{
		memset(m_cMilConnectorPinName, 0, sizeof(m_cMilConnectorPinName));
		strcpy(m_cMilConnectorPinName, "" );
	}
	return m_cMilConnectorPinName;
}

char* GetSpInternalSignalName(WORD wAdr, WORD wBitNum, char* strInOrOut)
{
	memset(m_cSpInternalSignalName, 0, sizeof(m_cSpInternalSignalName));
	if( wAdr >= SP_XXXX_SW1_START_ADR && wAdr <= 12 ){	//12:検査品の最終Adr
		sprintf(m_cSpInternalSignalName, "(%s%d-%x)", strInOrOut, (wAdr-SP_XXXX_SW1_START_ADR+1), wBitNum);
	}
	return m_cSpInternalSignalName;
}

////////////////////////////////////////////////////
//マルチメータから電圧値をRS232C経由で読み取る
BOOL GetVoltageVal(HANDLE hhandle, double *val)
{
	double	dwk1 = -1;
	BOOL	retGetMeasure_HP34401A;
	BOOL	berr=TRUE;

	//マルチメータから電圧値を取得する
	retGetMeasure_HP34401A = GetMeasure_HP34401A(
		hhandle			//通信リソースハンドル
		,&dwk1			//マルチメータ計測値
	);
	if( ! retGetMeasure_HP34401A ){
		LogPrintf("MultiMeter 34401A Data Read Error!!\n");
		berr = FALSE;
	}

	if( berr == TRUE ){
		//*val = dwk1 * 1000.0;	//取得値をV値に変換する
		*val = dwk1;
	}
	return berr;
}

////////////////////////////////////////////////////
// DA出力マルチメータ値の最小、最大、センター部分の平均を取得する
void GetDaMinMaxAve(double *ValArray, double *MinVal, double *MaxVal, double *AveVal)
{
	double	f_wk;
	int	ii, jj, nAveP;

	*AveVal = 0.0;
	*MaxVal = 0.0;
	*MinVal = 9999.0;

	for(ii=0; ii<DA_INPUT_DATA_NUM-1; ii++){
		for(jj=ii+1; jj<DA_INPUT_DATA_NUM; jj++){
			if(ValArray[ii] > ValArray[jj]){
				f_wk = ValArray[ii];
				ValArray[ii] = ValArray[jj];
				ValArray[jj] = f_wk;
			}
		}
	}
	*MinVal = ValArray[0];
	*MaxVal = ValArray[DA_INPUT_DATA_NUM-1];

	f_wk = 0;
	nAveP = (DA_INPUT_DATA_NUM-DA_SAMPL_NUM)/2;
	for(ii=0; ii<DA_SAMPL_NUM; ii++, nAveP++){
		f_wk = f_wk + ValArray[nAveP];
	}
	*AveVal = f_wk / (double)DA_SAMPL_NUM;
}

////////////////////////////////////////////////////
//アナログ入力の補正値（オフセット・ゲイン）を算出する
//#define	HARDWARE_FACTOR		(38496)		//ハード上の理論的な係数（0x9660）
//#define	HARDWARE_FACTOR_4MA	(8205)		//ハード上の理論的な4mA入力値（0x200D）
void GetAdOffsetGain(int nAdRange, DWORD AdAveVal4mA, DWORD AdAveVal20mA, DWORD *dwAdOffset, DWORD *dwAdGain)
{
	double d_wk;
	double d_gain_adj=0.0;

	double d_offset_factor = HARDWARE_FACTOR_4MA_OFFSET;
	double d_gain_factor = HARDWARE_FACTOR_20MA_GAIN;

	switch(nAdRange){
	case AD_INPUT_RANGE_4_20mA:
		d_offset_factor = HARDWARE_FACTOR_4MA_OFFSET;
		d_gain_factor = HARDWARE_FACTOR_20MA_GAIN;
		break;
	case AD_INPUT_RANGE_0_5V:
		d_offset_factor = HARDWARE_FACTOR_0V_OFFSET;
		d_gain_factor = HARDWARE_FACTOR_5V_GAIN;
		break;
	case AD_INPUT_RANGE_0_10V:
		d_offset_factor = HARDWARE_FACTOR_0V_OFFSET;
		d_gain_factor = HARDWARE_FACTOR_10V_GAIN;
		break;
	}

	//ゲイン
	*dwAdGain = 0;
	if( AdAveVal20mA > AdAveVal4mA ){
		d_wk = (double)(AdAveVal20mA - AdAveVal4mA);			//4mA → 20mA時の生データ変化幅

		d_wk = (double)(16384 * 65536) / d_wk;		//生データ変化幅を0～4000H幅に変換する係数
		
		d_gain_adj = (double)((double)(d_wk * 65536.0) / d_gain_factor);	//ゲイン補正値 

		d_wk = d_gain_adj - 65536.0 + 1024.0;
		if( d_wk < 0.0 ){
			*dwAdGain = 0x400;
		}
		else{
			*dwAdGain = (DWORD)(d_wk);
		}
	}
	else{
		printf("[WARNING] ADin [4mA_AVE=(0x%lX) > 20mA_AVE=(0x%lX)]\n", AdAveVal4mA, AdAveVal20mA);
	}

	//オフセット
	if( d_gain_adj > 0 ){
		d_wk = (double)(((double)AdAveVal4mA * d_gain_adj - (double)(d_offset_factor * 65536.0)) / d_gain_adj);
		//*dwAdOffset = (DWORD)(d_wk + 1024.0);
		d_wk = d_wk + 1024.0;
		if( d_wk < 0.0 ){
			*dwAdOffset = 0x400;
		}
		else{
			*dwAdOffset = (DWORD)(d_wk);
		}
	}
	else{
		printf("[WARNING] ADin [Gain Calc Val Invalid (0x%lX)]\n", (DWORD)d_gain_adj);
	}
}

////////////////////////////////////////////////////
// AD入力値の最小、最大、センター部分の平均を取得する
void GetAdMinMaxAve(DWORD *ValArray, DWORD *MinVal, DWORD *MaxVal, DWORD *AveVal)
{
	DWORD	dw_wk;
	int	ii, jj, nAveP;

	*AveVal = 0;
	*MaxVal = 0;
	*MinVal = 0xffff;

	for(ii=0; ii<AD_INPUT_DATA_NUM-1; ii++){
		for(jj=ii+1; jj<AD_INPUT_DATA_NUM; jj++){
			if(ValArray[ii] > ValArray[jj]){
				dw_wk = ValArray[ii];
				ValArray[ii] = ValArray[jj];
				ValArray[jj] = dw_wk;
			}
		}
	}
	*MinVal = ValArray[0];
	*MaxVal = ValArray[AD_INPUT_DATA_NUM-1];

	dw_wk = 0;
	nAveP = (AD_INPUT_DATA_NUM-AD_SAMPL_NUM)/2;
	for(ii=0; ii<AD_SAMPL_NUM; ii++, nAveP++){
		dw_wk = dw_wk + ValArray[nAveP];
	}
	*AveVal = dw_wk / AD_SAMPL_NUM;
}

////////////////////////////////////////////////////////////////////
//
//	ログファイル記録と画面表示
//
////////////////////////////////////////////////////////////////////
void LogPrintfCsv(const char *fname, const char * Format, ...)
{
	char	cLogName[_MAX_PATH];
	FILE* fp;
	va_list vl;

	va_start(vl, Format);

	// 画面表示
	vprintf(Format, vl);

	// ファイル保存
	if( lstrlen(m_cLogFolderName) > 0 ){
		if( lstrlen(m_strLogFileID) <= 0 ){
			if( lstrlen(m_strLotName) <= 0 ){
				sprintf(cLogName, "%s\\%s", m_cLogFolderName, fname);
			}
			else{
				sprintf(cLogName, "%s\\%s\\%s", m_cLogFolderName, m_strLotName, fname);
			}
		}
		else{
			if( lstrlen(m_strLotName) <= 0 ){
				sprintf(cLogName, "%s\\%s_%s", m_cLogFolderName, m_strLogFileID, fname);
			}
			else{
				sprintf(cLogName, "%s\\%s\\%s_%s", m_cLogFolderName, m_strLotName, m_strLogFileID, fname);
			}
		}
	}
	else{
		if( lstrlen(m_strLogFileID) <= 0 ){
			if( lstrlen(m_strLotName) <= 0 ){
				lstrcpy(cLogName, fname);
			}
			else{
				sprintf(cLogName, "%s\\%s", m_strLotName, fname);
			}
		}
		else{
			if( lstrlen(m_strLotName) <= 0 ){
				sprintf(cLogName, "%s_%s", m_strLogFileID, fname);
			}
			else{
				sprintf(cLogName, "%s\\%s_%s", m_strLotName, m_strLogFileID, fname);
			}
		}
	}
	fp = fopen(cLogName, "a");
	if(fp){
		vfprintf(fp, Format, vl);
		fclose(fp);
	}
	else{
		printf("\n<<%s File Open Error!!>>\a\a\n\n", cLogName);
	}
	va_end(vl);
}
	
void LogPrintf(const char * Format, ...)
{
	FILE* fp;

	va_list vl;

	va_start(vl, Format);

	// 画面表示
	vprintf(Format, vl);

	// ファイル保存
	fp = fopen("SPxxxxLog.txt", "a");
	if(fp){
		vfprintf(fp, Format, vl);
		fclose(fp);
	}

	va_end(vl);
}


//数値（文字列）入力待ち
void KeyStrInputWait(char *strbuf)
{
	KeyBufClear();		//キーバッファを空にする
	printf("\a");
	gets(strbuf);
}

//キー入力待ち
int KeyInputWait()
{
	int nKeyIn = 0;

	KeyBufClear();		//キーバッファを空にする
	printf("\a");
	nKeyIn = _getch();
	return nKeyIn;
}
int		KeyInputWaitYorN()
{
	int nKeyIn = 0;

	while(ForeverLoop){
		nKeyIn = KeyInputWait();
		if( nKeyIn == 'y' || nKeyIn == 'Y' || nKeyIn == 0x0d ){
			printf(" y\n");
			nKeyIn = 'y';
		}
		else if( nKeyIn >= '0' && nKeyIn <= '9' ){
			continue;		//数字が入力された場合は再入力待ちとする
		}
		else if( nKeyIn != 0x1B ){
			printf(" n\n");
			nKeyIn = 'n';
		}
		else{
			printf(" Esc\n");
		}
		break;
	}
	return nKeyIn;
}

//キー入力バッファクリア
void KeyBufClear()
{
	while(ForeverLoop){
		//キーバッファを空にする
		if( _kbhit() ){
			_getch();
		}
		else{
			break;
		}
	}
}
//何かキーの押下待ち
void	HitAnyKeyWait()
{
	KeyBufClear();
	_getch();
}
void	Sleep_Cnt( DWORD msec )
{
	if( m_nIniPerformanceCounterUsed != 0 ){
		LARGE_INTEGER	bef,aft;
		LARGE_INTEGER	val;

		QueryPerformanceCounter( &bef );
		while(ForeverLoop){
			QueryPerformanceCounter(&aft);
			val.QuadPart = aft.QuadPart - bef.QuadPart;
			if( val.QuadPart >= (msec * m_laMsecCnt.QuadPart ) ){
				break;
			}
		}
	}
	else{
		Sleep(msec);
	}
}

//ロットフォルダが無ければ生成する
void CreateLotFolder()
{
	int		nRet;
	char	cLogFolder[_MAX_PATH];

	if( lstrlen(m_strLotName) > 0 ){
		if( lstrlen(m_cLogFolderName) > 0 ){
			sprintf(cLogFolder, "%s\\%s", m_cLogFolderName, m_strLotName);
		}
		else{
			lstrcpy(cLogFolder, m_strLotName);
		}
		nRet = PathFileExists(cLogFolder);
		if( nRet == 0 )
		{	//パスが存在しないときは、フォルダを生成する
			if( CreateDirectory(cLogFolder, NULL) == 0 )
			{	//ディレクトリの生成に失敗した場合はログ出力しない
				printf("\n\a\a \"%s\" Folder cannot created.");
			}
		}
	}
}

void LogFolderCreate()
{
	//int	ii;
	//char cWk[256];
	int	nRet;
	unsigned long	ulRet;
	char szPath[_MAX_PATH+2];
	char szDrive[_MAX_PATH];
	char szDir[_MAX_PATH];
	char szFileName[_MAX_PATH];
	char szExt[_MAX_PATH];
	char szIni[_MAX_PATH];

	memset(m_cLogFolderName, 0, sizeof(m_cLogFolderName));
	strcpy(szIni, "Param.ini");

	ulRet = GetModuleFileName(NULL, szPath, _MAX_PATH);
	if( ulRet != 0 )
	{
		_splitpath(szPath, szDrive, szDir, szFileName, szExt);
		//wsprintf(m_cLogFolderName, "%s%sLog", szDrive, szDir);
		(void)_stprintf(m_cLogFolderName, "%s%sLog", szDrive, szDir);
		//(void)_stprintf(m_cLogFolderName, "%s%s", szDrive, szDir);

		nRet = PathFileExists(m_cLogFolderName);
		if( nRet == 0 )
		{	//パスが存在しないときは、フォルダを生成する
			if( CreateDirectory(m_cLogFolderName, NULL) == 0 )
			{	//ディレクトリの生成に失敗した場合はログ出力しない
				memset(m_cLogFolderName, 0, sizeof(m_cLogFolderName));
			}
		}
		
		//iniからRS-232C通信ポート番号を取得する
		(void)_stprintf(szIni, "%s%sParam.ini", szDrive, szDir);
	}
	GetPrivateProfileString("PARAM", "ComPort", "COM1", m_cComPort, sizeof(m_cComPort), szIni);
	GetPrivateProfileString("PARAM", "SPxxComPort", "COM2", m_cSpxxComPort, sizeof(m_cSpxxComPort), szIni);
	m_nIniBoardID = (int)GetPrivateProfileInt("PARAM", "SP_BOARD_TYPE_DEF", -1, szIni);
	m_nIniSpFrontBoardType = (int)GetPrivateProfileInt("PARAM", "SP_FRONT_IF_BOARD_TYPE", 1, szIni);	//DefaultはFront2
	m_nIniSpUpperBoardType = (int)GetPrivateProfileInt("PARAM", "SP_UPPER_IF_BOARD_TYPE", 1, szIni);	//DefaultはUpper2
	m_nIniSpMotherBoardType = (int)GetPrivateProfileInt("PARAM", "SP_MOTHER_BOARD_TYPE", 0, szIni);
	//m_nIniSpHlsTestDswAllOff = (int)GetPrivateProfileInt("PARAM", "SP_HLSTEST_DSW_ALL_OFF", 1, szIni);

	GetPrivateProfileString("PARAM", "AIO_UNIT_DEVICE_NAME", "AIO000", m_cIniAioUnitDeviceName, sizeof(m_cIniAioUnitDeviceName), szIni);

	//CONTEC AIOユニット用のパラメータを入力する(電圧出力)
	m_nIniAO_0_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AO_0_0V", 0, szIni);	
	m_nIniAO_2_5V = (int)GetPrivateProfileInt("AIO_UNIT", "AO_2_5V", 0, szIni);	
	m_nIniAO_5_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AO_5_0V", 0, szIni);	
	m_nIniAO_10_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AO_10_0V", 0, szIni);	
	m_nIniAO_0_OVR_L = (int)GetPrivateProfileInt("AIO_UNIT", "AO_0_OVR_L", 0, szIni);	
	m_nIniAO_5_OVR_H = (int)GetPrivateProfileInt("AIO_UNIT", "AO_5_OVR_H", 0, szIni);	
	m_nIniAO_10_OVR_H = (int)GetPrivateProfileInt("AIO_UNIT", "AO_10_OVR_H", 0, szIni);	

	//CONTEC AIOユニット用のパラメータを入力する(電圧入力)
	m_nIniAI_0_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AI_0_0V", 0, szIni);	
	m_nIniAI_10_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AI_10_0V", 0, szIni);	
	//0-10V入力直線補間の傾きを計算する
	m_dContecAiAdjGainV = 10.0 / (double)(m_nIniAI_10_0V-m_nIniAI_0_0V);
	//0-10V入力直線補間の切片を計算する
	m_dContecAiAdjOffsetV = 10.0 - (double)m_nIniAI_10_0V * m_dContecAiAdjGainV;

	//CONTEC AIOユニット用のパラメータを入力する(電流出力)
	m_nIniAO_4mA = (int)GetPrivateProfileInt("AIO_UNIT", "AO_4mA", 0, szIni);	
	m_nIniAO_12mA = (int)GetPrivateProfileInt("AIO_UNIT", "AO_12mA", 0, szIni);	
	m_nIniAO_20mA = (int)GetPrivateProfileInt("AIO_UNIT", "AO_20mA", 0, szIni);	
	m_nIniAO_4mA_OVR_L = (int)GetPrivateProfileInt("AIO_UNIT", "AO_4mA_OVR_L", 0, szIni);	
	m_nIniAO_20mA_OVR_H = (int)GetPrivateProfileInt("AIO_UNIT", "AO_20mA_OVR_H", 0, szIni);	

	//CONTEC AIOユニット用のパラメータを入力する(電流入力)
	m_nIniAI_4mA = (int)GetPrivateProfileInt("AIO_UNIT", "AI_4mA", 0, szIni);	
	m_nIniAI_20mA = (int)GetPrivateProfileInt("AIO_UNIT", "AI_20mA", 0, szIni);	
	//4-20mA入力直線補間の傾きを計算する
	m_dContecAiAdjGainI = (20.0 - 4.0) / (double)(m_nIniAI_20mA-m_nIniAI_4mA);
	//4-20mA入力直線補間の切片を計算する
	m_dContecAiAdjOffsetI = 20.0 - (double)m_nIniAI_20mA * m_dContecAiAdjGainI;
}

////////////////////////////////////////////////////////////////////
//
//          プログラム異常終了処理
//
////////////////////////////////////////////////////////////////////
void ErrorExit(DWORD ErrorCode)
{
    LogPrintf("Sample Program Terminated with Error!!\n");
	printf("\a\a\a");
    LogPrintf(" -> Error Code = ");
    switch(ErrorCode)
    {
    case SNMA_NO_ERROR:
        LogPrintf("No Error???\n");
        break;
    case SNMA_INIT_ERROR:
        LogPrintf("Failed to Initialize Board!!\n");
        break;
    case SNMA_INVALID_UNIT:
        LogPrintf("Invalid Unit Specified!! \n");
        break;
    case SNMA_REQ_INVALID_DATA:
        LogPrintf("Lower Level Driver Detected Bad Parameter!!\n");
        break;
    case SNMA_REQ_BAD_LENGTH:
        LogPrintf("Lower Level Driver Detected Bad Parameter Length!!\n");
        break;
    case SNMA_BAD_DEVICE:
        LogPrintf("Invalid Board ID || I/O Base Address Specified!!\n");
        break;
    default:
        LogPrintf("Unknown Error????? (%08X)\n",ErrorCode);
        break;
    }

    if(VxDOpen) {
        CloseVxD();                     // OpenVxD()が成功した場合、
                                        // 終了前にCloseVxDを必ず発行
    }
    FreeLibrary(hDll);                  // DLLを開放する

	printf("\nInput any key....");
	HitAnyKeyWait();
    exit(-1);                           // プログラム終了
}

////////////////////////////////////////////////////////////////////
//
//          ドライバDLLのロード、エントリー取得
//
////////////////////////////////////////////////////////////////////
BOOLEAN LoadSnDll(const char * pcFileName)
{
	char pcFile[32];

	if(pcFileName[0] == '\0'){
		return FALSE;
	}

	sprintf(pcFile, "%s.DLL", pcFileName);

	// DLLをロード
	hDll = LoadLibrary(pcFile);
	if (hDll == NULL) {
		LogPrintf("Load %s.DLL failed %08X\n", pcFileName, GetLastError());
		return(FALSE);
	}

	printf("Load %s.DLL Instance Handle = %08X\n", pcFileName, hDll);

    // Entry Pointの取得
    OpenVxD = (OPENVXD)GetProcAddress(hDll, TEXT("OpenVxD"));
    CloseVxD = (CLOSEVXD)GetProcAddress(hDll, TEXT("CloseVxD"));
    GETversion = (GETVERSION)GetProcAddress(hDll, TEXT("GETversion"));
    BOARDinit = (BOARDINIT)GetProcAddress(hDll, TEXT("BOARDinit"));
    TERMstart = (TERMSTART)GetProcAddress(hDll, TEXT("TERMstart"));
    TERMstop = (TERMSTOP)GetProcAddress(hDll, TEXT("TERMstop"));
    TERMchk = (TERMCHK)GetProcAddress(hDll, TEXT("TERMchk"));
    BOARDchk = (BOARDCHK)GetProcAddress(hDll, TEXT("BOARDchk"));
    UNITcntr = (UNITCNTR)GetProcAddress(hDll, TEXT("UNITcntr"));
    DATAin = (DATAIN)GetProcAddress(hDll, TEXT("DATAin"));
    DATAout = (DATAOUT)GetProcAddress(hDll, TEXT("DATAout"));
    DATAoutW = (DATAOUTW)GetProcAddress(hDll, TEXT("DATAoutW"));
    ADin = (ADIN)GetProcAddress(hDll, TEXT("ADin"));
    DAout = (DAOUT)GetProcAddress(hDll, TEXT("DAout"));
    CNTin = (CNTIN)GetProcAddress(hDll, TEXT("CNTin"));
    CNT0 = (CNT0_)GetProcAddress(hDll, TEXT("CNT0"));
    TERMread = (TERMREAD)GetProcAddress(hDll, TEXT("TERMread"));
    TERMwrite = (TERMWRITE)GetProcAddress(hDll, TEXT("TERMwrite"));
    RETRYread = (RETRYREAD)GetProcAddress(hDll, TEXT("RETRYread"));
    RETRYwrite = (RETRYWRITE)GetProcAddress(hDll, TEXT("RETRYwrite"));
    WIREread = (WIREREAD)GetProcAddress(hDll, TEXT("WIREread"));
    WIREwrite = (WIREWRITE)GetProcAddress(hDll, TEXT("WIREwrite"));
    EEPROMwrite = (EEPROMWRITE)GetProcAddress(hDll, TEXT("EEPROMwrite"));
    WIREswitch = (WIRESWITCH)GetProcAddress(hDll, TEXT("WIREswitch"));
    WIREon = (WIREON)GetProcAddress(hDll, TEXT("WIREon"));
    WIREoff = (WIREOFF)GetProcAddress(hDll, TEXT("WIREoff"));
    WIREchk = (WIRECHK)GetProcAddress(hDll, TEXT("WIREchk"));
    BOARDlist = (BOARDLIST)GetProcAddress(hDll, TEXT("BOARDlist"));
    TERMsetup = (TERMSETUP)GetProcAddress(hDll, TEXT("TERMsetup"));
    MltAdIn = (MLTADIN)GetProcAddress(hDll, TEXT("MltAdIn"));
    GetMltAdData = (GETMLTADDATA)GetProcAddress(hDll, TEXT("GetMltAdData"));

    // Entry Pointの確認
    if ((OpenVxD == NULL) || (CloseVxD == NULL) || (GETversion == NULL) || (BOARDinit == NULL) ||
        (TERMstart == NULL) || (TERMstop == NULL) || (TERMchk == NULL) || (BOARDchk == NULL) ||
        (UNITcntr == NULL) || (DATAin == NULL) || (DATAout == NULL) || (DATAoutW == NULL) ||
        (ADin == NULL) || (DAout == NULL) || (CNTin == NULL) || (CNT0 == NULL) ||
        (TERMread == NULL) || (TERMwrite == NULL) || (RETRYread == NULL) || (RETRYwrite == NULL) ||
        (WIREread == NULL) || (WIREwrite == NULL) || (EEPROMwrite == NULL) || (WIREswitch == NULL) ||
        (WIREon == NULL) || (WIREoff == NULL) || (WIREchk == NULL) || (BOARDlist == NULL) ||
        (TERMsetup == NULL) || (MltAdIn == NULL) || (GetMltAdData == NULL)) {

        // Entry Pointの取得に失敗した場合エラー終了
		LogPrintf("Failed to Get %s.DLL Entry Point!!\n", pcFileName);
        FreeLibrary(hDll);
        return(FALSE);
    }
    return(TRUE);
}

////////////////////////////////////////////////////////////////////
//
//	プログレスバー風画面表示
//
////////////////////////////////////////////////////////////////////
void WaitProgressDot(
	DWORD	Interval,	// ミリ秒間隔で.を表示
	DWORD	DotMax)		// . を表示する個数
{
	DWORD lp;

	for(lp = 0; lp < DotMax; lp++) {
		printf(".");
		Sleep_Cnt(Interval);
	}
}

////////////////////////////////////////////////////////////////////
//	CONTEC AIO-160802AY-USB関連の処理
////////////////////////////////////////////////////////////////////

///////////////////////////////////
//AIOユニット初期化
BOOL	AioUnitOpen()
{
	long	lAioRet;
	//float	fAiVal;
	//short	AiMaxChannels;
	//short	AoMaxChannels;

	//初期化
	if( m_shAioUnitId != -1 ){
		AioExit(m_shAioUnitId);
	}

	lAioRet = AioInit(m_cIniAioUnitDeviceName, &m_shAioUnitId);
    if(lAioRet != 0){
		AioGetErrorString(lAioRet , m_cAioStringWork);
		printf("AioInit(%ld):%s\n", lAioRet, m_cAioStringWork);
		printf("何かキーを押してください\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}
	AioResetDevice(m_shAioUnitId);	//デバイス設定を初期状態に戻す

	//入力レンジの設定
	lAioRet = AioSetAiRangeAll(m_shAioUnitId, PM10);	//AIO-160802AY-USBは±10V固定
	if(lAioRet != 0){
		AioGetErrorString(lAioRet, m_cAioStringWork);
		printf("AioSetAiRangeAll(%ld): %s\n", lAioRet, m_cAioStringWork);
		AioExit(m_shAioUnitId);
		printf("何かキーを押してください\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}
	
	//出力レンジの設定
	lAioRet = AioSetAoRangeAll(m_shAioUnitId, PM10);	//AIO-160802AY-USBは±10V固定
	if(lAioRet != 0){
		AioGetErrorString(lAioRet, m_cAioStringWork);
		printf("AioSetAoRangeAll(%ld): %s\n", lAioRet, m_cAioStringWork);
		AioExit(m_shAioUnitId);
		printf("何かキーを押してください\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}

	//アナログ入力チャネルの設定(使用するch数は8チャネル(MAX=8))
	//lAioRet = AioGetAiMaxChannels( m_shAioUnitId , &AiMaxChannels );
	lAioRet = AioSetAiChannels(m_shAioUnitId , 8);
	if(lAioRet != 0){
		AioGetErrorString(lAioRet, m_cAioStringWork);
		printf("AioSetAiChannels(%ld): %s\n", lAioRet, m_cAioStringWork);
		AioExit(m_shAioUnitId);
		printf("何かキーを押してください\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}

	//アナログ出力チャネルの設定(使用するch数は1チャネル(MAX=2))
	//lAioRet = AioGetAoMaxChannels( m_shAioUnitId , &AoMaxChannels );
	lAioRet = AioSetAoChannels(m_shAioUnitId , 1);
	if(lAioRet != 0){
		AioGetErrorString(lAioRet, m_cAioStringWork);
		printf("AioSetAoChannels(%ld): %s\n", lAioRet, m_cAioStringWork);
		AioExit(m_shAioUnitId);
		printf("何かキーを押してください\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}
	else{
		//初期出力は0Vとする
		AioSingleAo( m_shAioUnitId , 0 , m_nIniAO_0_0V );
	}
	//折り返しTest
	/*
	lAioRet = AioSingleAo( m_shAioUnitId , 0 , 0x0000 );
	Sleep_Cnt( 1000 );
	lAioRet = AioSingleAiEx( m_shAioUnitId , 0 , &fAiVal );	//-9.99847V

	lAioRet = AioSingleAo( m_shAioUnitId , 0 , 0x7000 );
	Sleep_Cnt( 1000 );
	lAioRet = AioSingleAiEx( m_shAioUnitId , 0 , &fAiVal );	//-1.25386V

	lAioRet = AioSingleAo( m_shAioUnitId , 0 , 0x8000 );
	Sleep_Cnt( 1000 );
	lAioRet = AioSingleAiEx( m_shAioUnitId , 0 , &fAiVal );	//0.00183105V

	lAioRet = AioSingleAo( m_shAioUnitId , 0 , 0xA000 );
	Sleep_Cnt( 1000 );
	lAioRet = AioSingleAiEx( m_shAioUnitId , 0 , &fAiVal );	//2.50153V
	
	lAioRet = AioSingleAo( m_shAioUnitId , 0 , 0xC000 );
	Sleep_Cnt( 1000 );
	lAioRet = AioSingleAiEx( m_shAioUnitId , 0 , &fAiVal );	//5.00153V
	
	lAioRet = AioSingleAo( m_shAioUnitId , 0 , 0xffff );
	Sleep_Cnt( 1000 );
	lAioRet = AioSingleAiEx( m_shAioUnitId , 0 , &fAiVal );	//9.99756V
	*/
	return TRUE;
}

///////////////////////////////////
//AIOユニット終了
void	AioUnitClose()
{
	if( m_shAioUnitId != -1 ){
		AioExit(m_shAioUnitId);
		m_shAioUnitId = -1;
	}
}

///////////////////////////////////
//AIOユニットに対して指定された電圧値を出力する
void	AioUnitWriteAOVal(short sAOch, long lAOval)
{
	BOOL	bOpenOK = TRUE;
	long	lAioRet;
	if( m_shAioUnitId == -1 ){
		bOpenOK = AioUnitOpen();
	}

	if( bOpenOK == TRUE ){
		//電圧出力を行う
		lAioRet = AioSingleAo( m_shAioUnitId, sAOch, lAOval );
		if(lAioRet != 0){
			AioGetErrorString(lAioRet, m_cAioStringWork);
			printf("AioSingleAo(%ld): %s\n", lAioRet, m_cAioStringWork);
			printf("何かキーを押してください\a\n");
			HitAnyKeyWait();
		}
	}
}

///////////////////////////////////
//AIOユニットから電圧値を読み出す
void	AioUnitReadAIVal(short sAIch, double *dAIval)
{
	BOOL	bOpenOK = TRUE;
	long	lAIval;
	long	lAioRet;

	*dAIval = 0.0;
	if( m_shAioUnitId == -1 ){
		bOpenOK = AioUnitOpen();
	}

	if( bOpenOK == TRUE ){
		//電圧値を読み出す
		lAioRet = AioSingleAi( m_shAioUnitId, sAIch, &lAIval );
		if(lAioRet != 0){
			AioGetErrorString(lAioRet, m_cAioStringWork);
			printf("AioSingleAo(%ld): %s\n", lAioRet, m_cAioStringWork);
			printf("何かキーを押してください\a\n");
			HitAnyKeyWait();
		}
		else{
			//読み出した値(16bit)を電圧値に変換する(直線補間)
			*dAIval = (double)((double)lAIval * m_dContecAiAdjGainV) + m_dContecAiAdjOffsetV;
		}
	}
}
