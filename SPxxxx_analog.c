//////////////////////////////////////////////////////////////////////////
//
//	SP_XXXX_BOARD �s������Ή��@�����v���O����
//
// --------�Ή��\��@��--------
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

//CONTEC AIO-160802AY-USB���C�u�������g�p����
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

#define	HARDWARE_FACTOR_20MA_GAIN		(0x5400)	//�n�[�h��̗��_�I�ȌW���iSP_FRONT_IF=0x5400�j
#define	HARDWARE_FACTOR_4MA_OFFSET		(0x35C7)	//�n�[�h��̗��_�I��4mA���͒l�iSP_FRONT_IF=0x35C7�j

#define	HARDWARE_FACTOR_10V_GAIN		(0x4472)	//�n�[�h��̗��_�I�ȌW���iSP_MOTHER�j
#define	HARDWARE_FACTOR_5V_GAIN			(0x4333)	//�n�[�h��̗��_�I�ȌW���iSP_UPPER�j
#define	HARDWARE_FACTOR_0V_OFFSET		(0x505)		//�n�[�h��̗��_�I��0V���͒l�iSP_UPPER_IF, SP_MOTHER�j

#define	INVALID_ADRAW_MINVAL_4mA		(0x3000-0x1000)
#define	INVALID_ADRAW_MAXVAL_4mA		(0x3000+0x1000)
#define	INVALID_ADRAW_MINVAL_20mA		(0xF860-0x1000)
#define	INVALID_ADRAW_MAXVAL_20mA		(0xFFFF)	//(0xF860+0x1000)

#define	INVALID_ADRAW_MINVAL_F0V		(0x0500-0x0500)		//0-5V(0V����)���l�̌x��臒l�͒������邱��
#define	INVALID_ADRAW_MAXVAL_F0V		(0x0500+0x1000)		//0-5V(0V����)���l�̌x��臒l�͒������邱��
#define	INVALID_ADRAW_MINVAL_F5V		(0xF430-0x1000)		//0-5V(5V����)���l�̌x��臒l�͒������邱��
#define	INVALID_ADRAW_MAXVAL_F5V		(0xFFFF)	//(0xF430+0x0500)		//0-5V(5V����)���l�̌x��臒l�͒������邱��

#define	INVALID_ADRAW_MINVAL_T0V		(0x0500-0x0500)		//0-10V(0V����)���l�̌x��臒l�͒������邱��
#define	INVALID_ADRAW_MAXVAL_T0V		(0x0500+0x1000)		//0-10V(0V����)���l�̌x��臒l�͒������邱��
#define	INVALID_ADRAW_MINVAL_T10V		(0xF430-0x1000)		//0-10V(10V����)���l�̌x��臒l�͒������邱��
#define	INVALID_ADRAW_MAXVAL_T10V		(0xFFFF)	//(0xF430+0x0500)		//0-10V(10V����)���l�̌x��臒l�͒������邱��

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
#define	SN_ADR_RELAY_OUT_UNIT		(55)			//�����[����p�^�[�~�i���A�h���X�i���j
typedef union{
	struct{
		unsigned short	B00B03_ANALOG_CH	:4;		//�A�i���O+��d�����o��ch (0x0:ch0�`0xF:ch15)
		unsigned short	B04_ANALOG_MODE_IN	:1;		//�o��(MultiMeter)���胂�[�h (0:�d������,1:�d������)
		unsigned short	B05_ANALOG_MODE_OUT	:1;		//D/A�o�͌������̓��͑��胂�[�h (0:�d������,1:�d������)��SP��ł�0:�d���o�͖͂���
		unsigned short	B06_ANALOG_RES_VAL	:1;		//D/A�o�͕��ג�R�؂�ւ�(0:���ׂȂ�,1:�d�����莞250���A�d�����莞1K��)��SP��ł�0:�d���o�͖͂���
		unsigned short	B07_ANALOG_IO_SL	:1;		//A/D���́AD/A�o�͌����؂�ւ�(0:D/A�o�́A1:A/D����)
		unsigned short	B08_PCB_PWR			:1;		//�������ON/OFF�؂�ւ�(0:OFF�A1:ON)
		unsigned short	B09_CPLD_SEL		:1;		//CPLD�̏������݃R�l�N�^�I��(0:CN101=HLS��,1:CN99=USR��)
		unsigned short	B10_TST1			:1;		//TST1�[�qON/OFF�؂�ւ�(0:OFF�A1:ON)
		unsigned short	B11_TST2			:1;		//TST2�[�qON/OFF�؂�ւ�(0:OFF�A1:ON)
		unsigned short	B12_TST3			:1;		//TST3�[�qON/OFF�؂�ւ�(0:OFF�A1:ON)
		unsigned short	B13_TST4			:1;		//TST4�[�qON/OFF�؂�ւ�(0:OFF�A1:ON)
		unsigned short	B14_E8A_CONNECT		:1;		//E8A�f�o�b�K�ڑ�(0:�ؒf�A1:�ڑ�)
		unsigned short	B15_HLS_CONNECT		:1;		//HLS�ʐM�ڑ�(0:�ؒf�A1:�ڑ�)
	}BIT;
	unsigned short	WORD;
}SN_CTL_WORD_BIT;
SN_CTL_WORD_BIT	m_snTestCtrlBit;

//�A�i���O���o��ch�؂�ւ�(�����[����)
#define	ANA_INOUT_SEL_IN		(0)		//���͌���
#define	ANA_INOUT_SEL_OUT		(1)		//�o�͌���
#define	ANA_VI_SEL_VOLT			(0)		//�d�����o�͌���
#define	ANA_VI_SEL_CURR			(1)		//�d�����o�͌���
#define	ANA_LOAD_SEL_OPEN		(0)		//��R���ׂȂ��iOPEN�j
#define	ANA_LOAD_SEL_RENBL		(1)		//��R���ׂ���
void SnMultiMeterMeasureSel(WORD wANAch, int nInOutSel, int nVISel, int nLoadSel);

//��d���ECPLD�I���EE8A�ڑ�/�ؒf�EHLS�ʐM���C���ڑ�/�f���؂�ւ�(�����[����)
void SnRelayOutputBit(int nBitSelect, WORD wData);		//1bit�̂ݐ؂�ւ��\��
void SnRelayOutputAllBit(WORD wPcbPwr, WORD wCpldSel, WORD wTst4En, WORD wE8aCon, WORD wHlsCon);	//�Sbit�����؂�ւ��\��

//��̓d���ē���
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
BOOLEAN	LoadSnDll(const char *);		// ��ײ��DLL��۰�ދy�Ѵ��؎擾
void ErrorExit(DWORD ErrorCode);		// �װ�I�����̏���
void WaitProgressDot(DWORD Interval, DWORD DotMax);
void SnDout16(int nOutMode, WORD wOutval, DWORD dwWait);

// �O���[�o���ϐ�
BrdList     ListBuf;                // BOARDlist()�Ŏg�p������Ǎ��݃o�b�t�@
WORD        BoardId;			    // Board ID
DWORD       termmap[64];		    // �^�[�~�i���}�b�s���O���̐錾
#endif

HINSTANCE   hDll = NULL;            // DLL�̃C���X�^���X�E�n���h��
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

//�{�[�hID�擾�E�ݒ�
#define	SP_BID_CHECK_ONLY			(0x000)
#define	SP_BID_UPPER_IF_BOARD		(0x155)
#define	SP_BID_FRONT_IF_BOARD		(0x146)
#define	SP_BID_MOTHER_BOARD			(0x14D)

#define SP_RS_MODE_NORMAL			('0')
#define SP_RS_MODE_ADADJ			('1')
#define SP_RS_MODE_DAADJ			('3')
#define SP_RS_MODE_RAW				('4')
#define SP_RS_MODE_EEPSAVE			('F')
#define SP_FIXED_SEND_SIZE			(39)	//���M�p�P�b�g�T�C�Y�i�Œ蒷�j
//#define SP_FIXED_RECV_SIZE			(40)	//��M�p�P�b�g�T�C�Y�i�Œ蒷�j
#define SP_FIXED_RECV_SIZE			(48)	//��M�p�P�b�g�T�C�Y�i�Œ蒷�j
#define SP_FIXED_RECV_SIZE_OLD		(40)	//��������@Ver�̃p�P�b�g�T�C�Y�i�Œ蒷�j
#define SP_XXXX_ADCH_MAX			(6)
#define SP_XXXX_DACH_MAX			(3)

#define INSMODE_NONE				(-1)	//�������[�h���͑҂�
#define INSMODE_MANUAL				(0)		//�蓮����
#define INSMODE_ALL					(1)		//�S����
#define INSMODE_AD_DA_ADJ			(2)		//AD,DA�����̂�
#define INSMODE_AD_DA_TEST			(3)		//AD,DA�����̂�

char		m_cLogFolderName[_MAX_PATH];	//���O�o�͐�t���p�X�t�@�C����
BOOLEAN     VxDOpen = FALSE;        // OpenVxD�t���O(TRUE -> open�ς�)
char		m_cComPort[128];				//
WORD        LastTermNum;			// �ŏI�^�[�~�i���A�h���X
WORD		m_wSnDoutData[4];
char		pcBoardName[32];		// �}�X�^�[�{�[�h��
int			m_nInspectMode = INSMODE_NONE;
char		m_strLogFileID[512];	//���O�t�@�C���̓�����
char		m_strLotName[512];		//���b�g���́i���O�ۑ���̃t�H���_���̂Ƃ���j
int			ForeverLoop=1;
int			m_nAutoSequence=-1;		//�S�e�X�g�V�[�P���X����t���O
DWORD		m_dwTestStartTick=0;		//�����J�n����TICK�J�E���^
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
BOOL SnDiReadWait(long *plnDiWaitVal, DWORD dwFirstWaitMsec);	//���̓f�[�^�����肵�ēǂݏo���邱�Ƃ��m�F����

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

//SAVENET��bit�����������MIL�R�l�N�^�s���ԍ��ɕϊ��i���b�Z�[�W�\���p�j
int GetPinNumMilTerm(WORD wBitNum);
char* GetPinNumMilTerm2(WORD wAdr, WORD wBitNum);

#define	SP_SIGNAL_OUT		(0)
#define	SP_SIGNAL_IN		(1)
char* GetSpInternalSignalName(WORD wAdr, WORD wBitNum, char* strInOrOut);

char m_cMilConnectorPinName[16];
char m_cSpInternalSignalName[16];


LARGE_INTEGER	m_laFreq;
LARGE_INTEGER	m_laMsecCnt;
void	Sleep_Cnt( DWORD msec );		//�p�t�H�[�}���X�J�E���^�𗘗p����Sleep�֐�

////////////////////////////////////////////////////////////////////
//
//          MAIN
//
////////////////////////////////////////////////////////////////////
void main( void )
{
#if (HLS_COMM==1)
	DWORD       err;
	WORD        version;            // ���ʃh���C�o�iSYS/VxD�o�[�W�����ԍ�)
	WORD        lpcnt;				// ���[�v�J�E���^
	DWORD       NumBoards;          // ���̃v���O���������s�����PC��ɑ��݂���}�X�^�[�{�[�h�̖���
	CfgInfo     *ListPtr;           // �{�[�h�R���t�B�O���[�V�������|�C���^
	DWORD       retrycnt;           // ���g���C�񐔐ݒ�l
#endif
	long	lBoardType, lCont;
	char	pcFileName[32];		// �t�@�C���� �g���q����������

	// Frequency�擾
	QueryPerformanceFrequency(&m_laFreq);
	m_laMsecCnt.QuadPart = m_laFreq.QuadPart / 1000;

    /////////////////////////////////////////////////
    // �ϐ�������
    VxDOpen = FALSE;                // OpenVxD�����t���O������
    LastTermNum = LAST_TERM_ADR;    // �ŏI�^�[�~�i���A�h���X������
	pcFileName[0] = '\0';
	memset(m_wSnDoutData, 0, sizeof(m_wSnDoutData));
	
    LogPrintf("**** SP_XXXX_BOARD �s������Ή� �����v���O���� ****\n");
    LogPrintf("            " __DATE__ "  " __TIME__ " Version\n");

	LogFolderCreate();

    /////////////////////////////////////////////////
	// �{�[�h�^�C�v�I��
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
		lBoardType = 2;		//SN-1002-PCIMA�Œ�
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
			return;	// �A�v���I��
		}
	}

	printf("\n");

    /////////////////////////////////////////////////
	//	�����^�[�~�i���ŏIID�����
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
    // DLL�����[�h�AAPI�֐��̃G���g�����擾����
    /////////////////////////////////////////////////
    if( !LoadSnDll(pcFileName) ) {
        // DLL�̃��[�h�܂���Entry�擾�Ɏ��s�����ꍇ�I��
        LogPrintf("Program Terminated with Error!! (hit any key.)\n");
		printf("\a");
		HitAnyKeyWait();
		return;
    }

    /////////////////////////////////////////////////
    // ���ׂĂ�API����ɐ旧����OpenVxD�𔭍s
    /////////////////////////////////////////////////
    err = OpenVxD();
    if(err != SNMA_NO_ERROR){
        LogPrintf("Failed to Excute <OpenVxD>  err = %08lx\n",err);
        ErrorExit(err);             // �G���[�I��
    } else {
        VxDOpen = TRUE;
    }

    /////////////////////////////////////////////////
    // ���ʃh���C�o�iSYS/VxD�j�o�[�W�����̎擾
    /////////////////////////////////////////////////
    err = GETversion(&version);
    if(err == SNMA_NO_ERROR) {
		LogPrintf("%s.SYS/VxD Version = %2X.%02X\n", pcFileName, (version & 0xff00)>> 8, version & 0x00ff);
    } else {
        LogPrintf("Failed to Get %s.SYS/VxD Version!! (%08X)\n", pcFileName, err);
        ErrorExit(err);             // �G���[�I��
    }

    /////////////////////////////////////////////////
    // SN-100X-PCIMA�{�[�h�̌��o
    /////////////////////////////////////////////////
    err = BOARDlist(&ListBuf, (LPINT *)&NumBoards);
    if(err == SNMA_INIT_ERROR){
        LogPrintf("Failed to Get %s Board List!!\n", pcBoardName);
        ErrorExit(err);             // �G���[�I��
    } else {
        LogPrintf("Number of %s Master Board : %02d\n", pcBoardName, (BYTE)NumBoards);
        ListPtr = (CfgInfo *)&ListBuf.Bd0Inf;           // �ŏ��̃{�[�h���̃|�C���^���Z�b�g
        for(lpcnt = 0;lpcnt < NumBoards;lpcnt ++)
        {
/***
            printf(" -------------------\n");
            printf("  Board ID = %02x\n",ListPtr->BoardId);     // �{�[�hID(DSW1�̐ݒ�l)
            printf("    Base I/O Address0(LCFG)         : %04x\n", ListPtr->CfgAddr);
            printf("    Base I/O Address1(%s): %04x\n", pcBoardName, ListPtr->BdAddr); // SN-1002-PCIMA�̎���
            printf("    IRQ Number                      : %02x\n", ListPtr->IrqNum); // �����Ă�ꂽIRQ
            printf("    Data Transfer Rate              : %-2dMbps\n", (1 << ListPtr->XferRate) * 3);
                                                                // ���݂̃f�[�^�`�����[�g�ݒ�l
            printf("    Data Transfer Mode              : ");   // ���݂̃f�[�^�`�����[�h�ݒ�
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
            switch(ListPtr->IrqEnb)                             // IRQ�o��Enable�ݒ�
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
        	ListPtr ++;                                 // ���̃{�[�h���ւ̃|�C���^��ݒ�
        }
    }
    BoardId = ListBuf.Bd0Inf.BoardId;               // �ŏ��Ɍ��������{�[�h�̃{�[�hID

	// �ʐM���~����
	err = TERMstop(BoardId);
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Stop Communication!!\n");
		ErrorExit(err);                     // �G���[�I��
	}
	// �{�[�h�̏�����
	err = BOARDinit(BoardId);
	if(err == SNMA_INIT_ERROR){
		LogPrintf("Failed to Initialize SN-1002-PCIMA Board!!\n");
		ErrorExit(err);                     // �G���[�I��
	}

	/////////////////////////////////////////////////
	// �ʐM���[�h�ݒ�(�`�����x�E�`�����[�g)
	/////////////////////////////////////////////////
	// SN-100X-PCIMA�̃h���C�o�C���X�g�[�����_�ł̒ʐM���[�h�ݒ菉���l�͈ȉ��̒ʂ�B
	//  �`�����x : 3Mbps
	//  �`�����[�h : Half Duplex
	//  ���V�X�e���N�����ɐݒ肳��鏉���ݒ�l.
	//  �������ݒ�l�̕ύX�́u�R���g���[���p�l���v�́uSAVE NET�v�A�v���b�g���N������
	//    �ύX���邱�Ƃ��ł��܂��B
	//  ��1998�N11�����ݎ��_�ł͕��ЕW�����i�̃^�[�~�i���i���W���[���j�͑S��3Mbps
	//    / Half Duplex�g�p�ƂȂ��Ă���ׁA���̕����̑���͕K�v����܂���B
	//
	//  �V�X�e���N����ɏ����ݒ�l����ȉ��̕��@�ŒʐM�ݒ��ύX���邱�Ƃ���
	//  ���܂��B
	//

	err = TERMsetup(BoardId, Xfer6M, HalfDup, EnbIrq);
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Setup Communication Mode!!\n");
		ErrorExit(err);                     // �G���[�I��
	}
//	LogPrintf("TERMsetup Xfer3M HalfDup EnbIrq\n");

	/////////////////////////////////////////////////
	// ���g���C�񐔐ݒ�E�擾
	/////////////////////////////////////////////////

	retrycnt = 1;                                   // �ő�̐ݒ�l��7
	err = RETRYwrite(BoardId,(LPINT *)&retrycnt);   // ���g���C�񐔏�����
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Set Retry Count!!\n");
		ErrorExit(err);             // �G���[�I��
	}
	err = RETRYread(BoardId,(LPINT *)&retrycnt);    // ���g���C�񐔓Ǎ�
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Retrieve Retry Count!!\n");
		ErrorExit(err);             // �G���[�I��
	}

	/////////////////////////////////////////////////
	// �^�[�~�i���E�}�b�v��� �ݒ�
	termmap[0] = Unused;		// �}�b�v���𖢐ڑ��Ƃ���
	for(lpcnt = 1; lpcnt <= LastTermNum; lpcnt++) {
		termmap[lpcnt] = Io8;		// �^�[�~�i���E�A�h���X n����o�͂ɐݒ�
	}
	for( ; lpcnt < 64; lpcnt ++) {
		termmap[lpcnt] = Unused;	// �}�b�v���𖢐ڑ��Ƃ���
	}

	err = TERMwrite(BoardId, (LPINT *)&termmap[0]);  // �^�[�~�i���E�}�b�s���O��񏑍���
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Store Terminal Mapping Information!!\n");
		ErrorExit(err);             // �G���[�I��
	}
	err = EEPROMwrite(BoardId);         // EEPROM�Ƀf�[�^����������
										// SN-1002-PCIMA�̏ꍇ���ۂɂ̓��W�X�g���ɓo�^�B
										// ���̊֐������s����Ȃ���TERMwrite()�ɂ��
										// �f�[�^�ݒ�͎��ۂ̓���ɔ��f����Ȃ��B
	if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Store EEPROM Information!!\n");
		ErrorExit(err);             // �G���[�I��
	}

	//printf("SAVENET�ʐM���J�n���܂�");

	Sleep_Cnt(300);
    err = TERMstart(BoardId);               // �{�[�h�̒ʐM���J�n����
    if(err != SNMA_NO_ERROR){
		LogPrintf("Failed to Start Communication!!\n");
		ErrorExit(err);             // �G���[�I��
    }

	WaitProgressDot(200, 10);
	printf("\n\n");
#endif

	//SPxxxxAdjMain();		//�������C�����[�v
	SPxxxxMain();

#if (HLS_COMM==1)
    err = TERMstop(BoardId);			// �{�[�h�̒ʐM���~����
    if(err != SNMA_NO_ERROR){
        LogPrintf("Failed to Stop Communication!!\n");
		ErrorExit(err);					// �G���[�I��
    }

	CloseVxD();                         // �I���O��CloseVxD��K�����s
	VxDOpen = FALSE;

    FreeLibrary(hDll);                  // DLL���J������
#endif
	
	if( m_hSpComHandle != INVALID_HANDLE_VALUE ){
		CloseHandle(m_hSpComHandle);
		m_hSpComHandle = INVALID_HANDLE_VALUE;
	}
	
	AioUnitClose();		//CONTEC AIO���j�b�g�p�h���C�o��CLOSE����
	
	printf("\nHit any key.\n");
	HitAnyKeyWait();
}

//////////////////////
//SAVENET �����o��
//�������(#4�`#12),�����@��(#48�`#58)��DO�o�͂�CLR����
//�� #55�̃����[���䃆�j�b�g��CLR����ꍇ�́A����bAllUnit��TRUE���w�肷��
void SnOutputInit(BOOL bAllUnit)
{
	WORD	ii;

	//��������̏o�͂�CLR
	for( ii=SP_XXXX_SW1_START_ADR; ii<=SP_XXXX_SW1_START_ADR+8; ii++ ){
		DATAout(BoardId, ii, 0x0000);
	}

	//�����@���̏o�͂�CLR
	for( ii=ADR_DO_START; ii<=ADR_DO_END; ii++ ){
		if( ii == SN_ADR_RELAY_OUT_UNIT ){	//�����[���䃆�j�b�g(#55)
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
//���������ini�t�@�C������擾����
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
//			SP_XXXX_BOARD �����E�������C��
//			�s������ɂ�钲������
////////////////////////////////////////////////////////////////////
void	SPxxxxMain()
{
	int	nInputVal = -1;

	while(ForeverLoop){
		SPxxxxGetBoardName();	//��������擾
		SnOutputInit(TRUE);		//SAVENET �����o�͑SCLR

		system("cls");			//��ʃN���A
		printf("-------------------------------------------------------\n");
		printf(" <<<  %s �����p�\�t�g (Software Ver.%s)   >>>\n", m_cSPxxxxBoardName, SOFTWARE_VERSION);
		printf("-------------------------------------------------------\n");
		printf("<<<<<<<<<<<<<<<<   �������ڑI�����j���[  >>>>>>>>>>>>>>>>\n");
		printf("[1]:�S�����J�n\n");
		printf("[2]:�ʌ������j���[\n");
		printf("[Esc]:�����\�t�g�I��\n");
		printf("\n");

		printf("\n�����@�̓d���𓊓���A���j���[��I�����Ă�������-->");
	
		//�L�[���͑҂�
		nInputVal = KeyInputWait();
		if( nInputVal == 0x1B ){
			break;		//���C�����j���[�ɖ߂�
		}
		else{
			printf("%c\n", nInputVal);
			//���̓L�[�̊m�F
			switch(nInputVal){
			case (int)'1':	//�S�����J�n
				SPxxxxAllTestStart();
				break;
			case (int)'2':	//�ʌ������j���[
				SPxxxxTestSelectMenu();
				break;
			default:
				break;
			}
		}
	}
}

//�S�����J�n
void SPxxxxAllTestStart()
{
	BOOL	berr;
	system("cls");		//��ʃN���A
	berr = SPxxxxSequenceTest('1', 'Z');

	//system("cls");		//��ʃN���A
	printf("\n\n\a");
	printf("_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/\n");
	if( berr == TRUE ){
		printf("  [�������ʁFOK] �����L�[�������Ă��������B\n");
	}
	else{
		printf("  [�������ʁFNG] �����L�[�������Ă��������B\a\a\a\n");
	}
	printf("_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/\n");
	HitAnyKeyWait();
}

//////////////////////////////////////////////////////////
//SPxxxx��̃p�����[�^�Ŏw�����ꂽ���������Ɏ��{����
BOOL SPxxxxSequenceTest(int nInsStartID, int nInsEndID)
{
	int	ii;
	int nInputVal=-1;
	BOOL bErr = TRUE;
	BOOL bReturn = TRUE;

	//�I�����ꂽ�������ڂ̌��������Ɏ��s����('1'��'9', 'A'��'Z')
	for(ii=nInsStartID; ii<=nInsEndID; ii++){
		if( (ii > '9') && (ii < 'A') ){
			ii = 'A';
		}
		system("cls");		//��ʃN���A
	
		//�����O�̏��������[�o�́i��d��ON�ACPLD(CN101��)�ɐ؂�ւ��ATST4�[�q�o�́AE8A�ڑ��f�AHLS�ڑ��j
		SnRelayOutputAllBit(1, 0, 0, 0, 1);

		switch(ii){
		case (int)'1':		//�d���d���m�F
			bErr = SPxxxxDC_PwrChk(ii);
			break;
		case (int)'2':		//�t�@�[���E�F�A��������
			bErr = SPxxxxFirmWrite(ii);
			break;
		case (int)'3':		//�����pCPLD��������(SPIN,SP_MOTHER�̂�)
			if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
				bErr = SPxxxxCPLDWrite(ii, 0);
			}
			break;
		case (int)'4':		//LED�E����d���ڎ��m�F
			bErr = SPxxxxLedTest(ii);
			break;
		case (int)'5':		//�{�[�hID�ݒ�
			RS_SpComOpen();		//RS232C�ʐM�|�[�g���I�[�v������
			PcbPwrReBoot(TRUE);		//��������ċN������
			bErr = SPxxxxBIDSetting(ii);
			break;
		case (int)'6':		//HLS�ʐM�e�X�g(NET1�R�l�N�^��)
			bErr = SPxxxxComChk(ii, 0);
			break;
		case (int)'7':		//�f�W�^�����o�͕����e�X�g�i�@�했�Ɍʏ����j
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
		case (int)'8':		//�f�W�^�����o�͐܂�Ԃ��e�X�g(Auto)
			bErr = SPxxxxDioChkAuto(ii, TRUE, TRUE);
			break;
		case (int)'A':		//�A�i���O���͒���
			if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ||
				m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ||
				m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
				RS_SpComOpen();		//RS232C�ʐM�|�[�g���I�[�v������
				PcbPwrReBoot(TRUE);		//��������ċN������
				bErr = SPxxxxAdInAdj(ii, -1, -1);
			}
			break;
		case (int)'B':		//�A�i���O�o�͒���
			if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
				RS_SpComOpen();		//RS232C�ʐM�|�[�g���I�[�v������
				PcbPwrReBoot(TRUE);		//��������ċN������
				//bErr = SPxxxxDaOutAdj		//SPIN_MOTHER�ł͎g�p���Ȃ��̂Ō�܂킵
			}
			break;
		case (int)'C':		//�A�i���O���͐��x�m�F
			if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ||
				m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ||
				m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
				bErr = SPxxxxAdinChk(ii, -1, -1);
			}
			break;
		case (int)'D':		//�A�i���O�o�͐��x�m�F
			if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
				//SPxxxxDaoutChk	//SPIN_MOTHER�ł͎g�p���Ȃ��̂Ō�܂킵
			}
			break;

		case (int)'E':		//���i�pCPLD��������(SPIN,SP_MOTHER�̂�)
			if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
				bErr = SPxxxxCPLDWrite(ii, 1);
			}
			break;
		case (int)'F':		//HLS�ʐM�e�X�g(NET2�R�l�N�^��)
			bErr = SPxxxxComChk(ii, 1);
			break;

		default:
			break;
		}

		if( bErr != TRUE ){
			bReturn = FALSE;
			if( ii == nInsEndID ){
				//�ŏI�����̂Ƃ�
				break;
			}
			else{
				//�����r���̂Ƃ�
				printf("�����𒆎~���܂����H(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					printf("\n--------------------------");
					printf("\n---�����𒆎~���܂����I---");
					printf("\n--------------------------\a\n\n");
					break;
				}
			}
		}

		//RS232C�ʐM�|�[�g��CLOSE��A��������ċN������
		if( m_hSpComHandle != INVALID_HANDLE_VALUE ){
			CloseHandle(m_hSpComHandle);
			m_hSpComHandle = INVALID_HANDLE_VALUE;
			PcbPwrReBoot(FALSE);		//��������ċN������
		}
	}	//for end

	return bReturn;
}

//////////////////////////////////////////
//�ʌ������j���[
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
		SnOutputInit(FALSE);		//SAVENET �����o��

		system("cls");		//��ʃN���A
		printf("<<<<<<<<<<<  �ʌ������j���[  >>>>>>>>>\n");
		printf("[1]:�d���d���m�F\n");
		printf("[2]:�t�@�[���E�F�A��������\n");
		if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[3]:�����pCPLD��������\n");
		}
		printf("[4]:LED�_���^����d���m�F\n");
		printf("[5]:�{�[�hID�ݒ�\n");
		printf("[6]:HLS�ʐM�e�X�g(NET1�R�l�N�^��)\n");

		if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[7]:�f�W�^�����o�͕����e�X�g\n");
		}
		printf("[8]:�f�W�^�����o�͐܂�Ԃ��e�X�g\n");
		
		//printf("\n");

		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ||
			m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ||
			m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[A]:�A�i���O���͒���\n");
		}

		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
			printf("[B]:�A�i���O�o�͒���\n");
		}

		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ||
			m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ||
			m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[C]:�A�i���O���͐��x�m�F\n");
		}

		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
			printf("[D]:�A�i���O�o�͐��x�m�F\n");
		}

		if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			printf("[E]:�����pCPLD��������\n");
		}
		printf("[F]:HLS�ʐM�e�X�g(NET2�R�l�N�^��)\n");
		printf("\n[Esc]:���C�����j���[�ɖ߂�\n");

		printf("\n���j���[��I�����Ă�������-->");

		//�L�[���͑҂�
		nInputVal = KeyInputWait();
		if( nInputVal == 0x1B ){
			break;		//�ʌ����I��
		}
		else{
			if( nInputVal >= (int)'a' && nInputVal <= (int)'z' ){
				nInputVal = nInputVal - 0x20;		//�������A���t�@�x�b�g�͑啶���ɕϊ����ď�������
			}
			printf("%c\n", nInputVal);
			//���̓L�[�̊m�F
			switch(nInputVal){
			case (int)'1':		//�d���d���m�F
			case (int)'2':		//�t�@�[���E�F�A��������
			case (int)'3':		//�����pCPLD��������(SPIN,SP_MOTHER�̂�)
			case (int)'4':		//LED�_���^����d���m�F
			case (int)'5':		//�{�[�hID�ݒ�
			case (int)'6':		//HLS�ʐM�e�X�g(NET1�R�l�N�^��)
			case (int)'7':		//�f�W�^�����o�͕����e�X�g
			case (int)'8':		//�f�W�^�����o�͐܂�Ԃ��e�X�g
			case (int)'A':		//�A�i���O���͒���
			case (int)'B':		//�A�i���O�o�͒���
			case (int)'C':		//�A�i���O���͐��x�m�F
			case (int)'D':		//�A�i���O�o�͐��x�m�F
			case (int)'E':		//���i�pCPLD��������(SPIN,SP_MOTHER�̂�)
			case (int)'F':		//HLS�ʐM�e�X�g(NET2�R�l�N�^��)
				nInsStartID = nInputVal;
				nInsEndID = nInputVal;
				break;
			default:
				nInsStartID = -1;
				nInsEndID = -1;
				bInspectSkip = TRUE;		//�����Ȃ�
				break;
			}

			if( bInspectSkip != TRUE ){
				if( nInsStartID != PCB_TEST_LAST_CODE ){
					printf("[%c]�ȍ~�������āA�������܂����H(y or n) -->", nInsStartID);
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						nInsEndID = PCB_TEST_LAST_CODE;
					}
				}

				bErr = SPxxxxSequenceTest(nInsStartID, nInsEndID);	//�ʌ������J�n����
				//system("cls");		//��ʃN���A
				printf("\n\n\a");
				printf("_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/\n");
				if( bErr == TRUE ){
					printf("  [�������ʁFOK] �����L�[�������Ă��������B\n");
				}
				else{
					printf("  [�������ʁFNG] �����L�[�������Ă��������B\a\a\a\n");
				}
				printf("_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/\n");
				HitAnyKeyWait();
			}
		}
	}	//while end
}

/////////////////////////////////////
//��d���ECPLD�I���EE8A�ڑ�/�ؒf�EHLS�ʐM���C���ڑ�/�f���؂�ւ�(�����[����)
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
			printf("�����L�[�������Ă��������B\a\n");
			HitAnyKeyWait();
		}
		else{
			Sleep_Cnt(100);
		}
	}
}
//0�F�Y��bit OFF�A1�F�Y��bit ON�A0/1�ȊO�F�Y��bit�̐ؑւȂ�
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
			printf("�����L�[�������Ă��������B\a\n");
			HitAnyKeyWait();
		}
		else{
			Sleep_Cnt(100);
		}
	}	
}

////////////////////////////
//������̓d���ē���
// bRsMode��TRUE�w�莞�́ATST4�[�q��ON�iRS-232C�ʐM���[�h�j��MPU���N������
void PcbPwrReBoot(BOOL bRsMode)
{
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 0);	//�d��OFF
	if( bRsMode == TRUE ){
		//��d��OFF�ACPLD(CN101��)[�ؑւȂ�]�ATST4�[�q�o��ON�AE8A�ڑ�[�ؑւȂ�]�AHLS�ڑ�[�ؑւȂ�]
		SnRelayOutputAllBit(0, 0xffff, 1, 0xffff, 0xffff);
	}
	else{
		//��d��OFF�ACPLD(CN101��)[�ؑւȂ�]�ATST4�[�q�o��OFF�AE8A�ڑ�[�ؑւȂ�]�AHLS�ڑ�[�ؑւȂ�]
		SnRelayOutputAllBit(0, 0xffff, 0, 0xffff, 0xffff);
	}
	Sleep_Cnt(500);	//�����҂�
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 1);	//�d��ON
	Sleep_Cnt(500);	//�����҂�
}

/////////////////////////////////////
//�A�i���O���o��ch�؂�ւ�(�����[����)
void	SnMultiMeterMeasureSel(WORD wANAch, int nInOutSel, int nVISel, int nLoadSel)
{
	DWORD	err;

	if( BoardId != 0xffff ){
		m_snTestCtrlBit.BIT.B00B03_ANALOG_CH = wANAch;	//�A�i���Och�ԍ����w��(ch4:DC5V, ch5:DC24V)
		if( nVISel == ANA_VI_SEL_VOLT ){
			//�d�����o�̓��[�h
			m_snTestCtrlBit.BIT.B04_ANALOG_MODE_IN = 1;		//�}���`���[�^�ڑ���d������p�ɐ؂�ւ���
			m_snTestCtrlBit.BIT.B05_ANALOG_MODE_OUT = 1;	//D/A�o�͂�d���o�̓��[�h�ɐ؂�ւ���
		}
		else{
			//�d�����o�̓��[�h
			m_snTestCtrlBit.BIT.B04_ANALOG_MODE_IN = 0;		//�}���`���[�^�ڑ���d������p�ɐ؂�ւ���
			if( nInOutSel == ANA_INOUT_SEL_OUT ){		//D/A�o�͌������[�h��
				//SPxxxx��ł͓d���o�͖͂����̂ŁA�����ɓ��邱�Ƃ͖���
				m_snTestCtrlBit.BIT.B05_ANALOG_MODE_OUT = 0;	//D/A�o�͂�d���o�̓��[�h�ɐ؂�ւ���
			}
			else{										//A/D���͌������[�h��
				m_snTestCtrlBit.BIT.B05_ANALOG_MODE_OUT = 1;	//D/A�o�͂�d���o�̓��[�h�ɐ؂�ւ���
			}
		}

		//��R���ׂ̗L��
		if( nLoadSel == ANA_LOAD_SEL_OPEN ){
			m_snTestCtrlBit.BIT.B06_ANALOG_RES_VAL = 0;		//�d���o�͂̒�R���ׂ�OPEN�ɂ���
		}
		else{
			m_snTestCtrlBit.BIT.B06_ANALOG_RES_VAL = 1;		//�d���o�͂̒�R���ׂ�L���ɂ���(�d���o��:250���A�d���o��:1K��)
		}

		//�A�i���O����or�A�i���O�o��
		if( nInOutSel == ANA_INOUT_SEL_OUT ){		//D/A�o�͌������[�h��
			m_snTestCtrlBit.BIT.B07_ANALOG_IO_SL = 0;		//�}���`���[�^�������@�����̃A�i���O�o�͌��������ɐڑ�����
		}
		else{
			m_snTestCtrlBit.BIT.B07_ANALOG_IO_SL = 1;		//�}���`���[�^�������@�����̃A�i���O���͌��������ɐڑ�����
		}

		err = DATAout(BoardId, SN_ADR_RELAY_OUT_UNIT, (WORD)m_snTestCtrlBit.WORD);
		if( err != SNMA_NO_ERROR ){
			printf("DATAout() err=%ld\n", err);
			printf("�����L�[�������Ă��������B\a\n");
			HitAnyKeyWait();
		}
		else{
			Sleep_Cnt(100);
		}
	}
}

//////////////////////////////////////////
//�f�W�^���}���`���[�^���g�p���Č�����̓d���d���͈͂̃`�F�b�N���s��
#define		RATE5PER					(5.0)
#define		RATE3PER					(3.0)
#define		RATE05PER					(0.5)
//#define		RATE_D12PLUSPER				(10.0)	//D12V�̃v���X�����e���x
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
		"\n������ %c:DC�d���d���m�F %s %s START ������\n", nInsID, strDate, strTime);

	//�f�W�^���}���`���[�^�̏���
	retStart_HP34401A = Start_HP34401A(
		m_cComPort,
		&forMultiMaterHandles	//�ʐM���\�[�X�̃n���h����Ԃ��A�h���X
	);
	if( ! retStart_HP34401A ){
		printf("MultiMeter 34401A Initialize Error!!\n");
		return FALSE;
	}

	//�}���`���[�^�̃����W��d�����͂ɐݒ肷��
	retChangeVA_DC_HP34401A = ChangeVoltageDC_HP34401A(
		forMultiMaterHandles	//�ʐM���\�[�X�n���h��
	);
	if( !retChangeVA_DC_HP34401A ){
		printf("MultiMeter 34401A Voltage Setting Error!!\n");
		End_HP34401A(forMultiMaterHandles);	//�ʐM���\�[�X�̃n���h�������
		return FALSE;
	}

	if( bRet == TRUE ){
		while(ForeverLoop){
			for( ii=1; ii<=DC_PWR_RANGE_CHK_NUM; ii++ ){
				m_DcPwrChkTable[ii].fRangeMinV = m_DcPwrChkTable[ii].fStdV - 
					( m_DcPwrChkTable[ii].fStdV * (m_DcPwrChkTable[ii].fRatePer/100.0));

					m_DcPwrChkTable[ii].fRangeMaxV = m_DcPwrChkTable[ii].fStdV + 
						( m_DcPwrChkTable[ii].fStdV * (m_DcPwrChkTable[ii].fRatePer/100.0));

				//����d���p�����[(�A�i���Och)��؂�ւ���
				SnMultiMeterMeasureSel(m_DcPwrChkTable[ii].wChSel, ANA_INOUT_SEL_OUT, ANA_VI_SEL_VOLT, ANA_LOAD_SEL_OPEN);
			
				//����J�n�i�}���`���[�^�̏ꍇ�j
				if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
					printf("[NG] �d���l���擾�ł��܂���ł����B�����L�[�������Ă��������B\a\n");
					HitAnyKeyWait();
					bRet = FALSE;
					break;
				}
				else{
					if( (fVoltageVal < m_DcPwrChkTable[ii].fRangeMinV) ||
						(fVoltageVal > m_DcPwrChkTable[ii].fRangeMaxV) ){
						//�d���l�ُ�
						LogPrintfCsv(m_strLogFileName, 
							"[NG] %8s: [%6.3f]V, [����͈�(�}%3.1f%%/FS)=%6.3fV�`%6.3fV]\n", 
							m_DcPwrChkTable[ii].cSignalName, fVoltageVal,
							m_DcPwrChkTable[ii].fRatePer,
							m_DcPwrChkTable[ii].fRangeMinV, m_DcPwrChkTable[ii].fRangeMaxV);
						bRet = FALSE;		//�d���l�ُ�
					}
					else{
						//�d���l����
						LogPrintfCsv(m_strLogFileName, 
							"[OK] %8s: [%6.3f]V, [����͈�(�}%3.1f%%/FS)=%6.3fV�`%6.3fV]\n", 
							m_DcPwrChkTable[ii].cSignalName, fVoltageVal,
							m_DcPwrChkTable[ii].fRatePer,
							m_DcPwrChkTable[ii].fRangeMinV, m_DcPwrChkTable[ii].fRangeMaxV);
					}
				}
			}	//for end
			//����d����NC�ɖ߂�
			SnMultiMeterMeasureSel(SN_MM_SEL_NC, ANA_INOUT_SEL_IN, ANA_VI_SEL_VOLT, ANA_LOAD_SEL_OPEN);

			if( bRet != TRUE ){
				printf("�Č������܂����H(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					bRet = TRUE;
					LogPrintfCsv(m_strLogFileName, "\n----�Č���----\n");
					continue;			//�Č������{
				}
			}
			break;
		}	//while(�Č����p) end 
	}
	
	/////////////////////////////////////////////////////////////////////
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"DC�d���d���m�F���ʁF%s\n", ((bRet==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	//�}���`���[�^�ʐM���\�[�X�̃n���h�������
	if( forMultiMaterHandles != NULL ){
		End_HP34401A(forMultiMaterHandles);
	}

	/////////////////////////////////////////////////////////////////////
	//�I������
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:DC�d���d���m�F %s %s FINISH ������\n", nInsID, strDate, strTime);

	return bRet;
}

//////////////////////////////////////////
//SP_XXXX���E8A���g�p���ăt�@�[���E�F�A����������
// (F/W�̏������݃\�t�g�� HEW���g�p)
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
		"\n������ %c:�t�@�[���E�F�A�������� %s %s START ������\n", nInsID, strDate, strTime);

	//�����i�̓d����OFF
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 0);	//�d��OFF
	Sleep_Cnt(500);	//�����҂�

	//E8A�ڑ�(�����[�؂�ւ�)
	SnRelayOutputBit(BIT_SEL_E8ACON_CTRL, 1);	//E8A�ڑ�

	//�����i�̓d����ON
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 1);	//�d��ON
	
	//HEW���g�p���ăt�@�[���E�F�A����������
	printf("\n");
	printf("------------------------------------------------------\n");
	printf("  ��CN100(�t�@�[���E�F�A�̏������݁�\n");
	printf("   HEW ���g�p���ăt�@�[���E�F�A�̏������݂��J�n���Ă��������B\n");
	printf("------------------------------------------------------\n");
	printf("\n");
	
	printf("�I������ꍇ�́A'Y'(����I��)�܂��́A'N'(�ُ�I��)����͂��Ă�������\n");
	while(ForeverLoop){
		nInputVal = KeyInputWait();
		if( nInputVal == 'y' || nInputVal == 'Y' || nInputVal == 0x0d ){
			//����I��
			break;
		}
		else if( nInputVal == 'n' || nInputVal == 'N' || nInputVal == 0x1B ){
			//�ُ�I��
			bRet = FALSE;
			break;
		}
		else{	//���̑��̃L�[�͖�������
			continue;
		}
	}

	//LogPrintfCsv(m_strLogFileName,"  [%s]: CN100 �t�@�[���E�F�A�̏�������\n", ((bRet==TRUE) ? "OK":"NG"));
	
	//�����i�̓d����OFF
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 0);
	Sleep_Cnt(500);	//�����҂�
	
	//E8A�ؒf
	SnRelayOutputBit(BIT_SEL_E8ACON_CTRL, 0);
	
	//�����i�̓d����ON
	SnRelayOutputBit(BIT_SEL_PCBPWR_CTRL, 1);

	
	/////////////////////////////////////////////////////////////////////
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"�t�@�[���E�F�A�������݁F%s\n", ((bRet==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	/////////////////////////////////////////////////////////////////////
	//�I������
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:�t�@�[���E�F�A�������� %s %s FINISH ������\n", nInsID, strDate, strTime);

	return bRet;
}

//////////////////////////////////////////
//SP_MOTHER, SPIN_MOTHER�p��CPLD����������
// (CPLD�̏������݃\�t�g�� QuatusII���g�p)
//nCpldId=0: �����p CPLD��������
//nCpldId=1: ���i�p CPLD��������
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
		"\n������ %c:CPLD�������� %s %s START ������\n", nInsID, strDate, strTime);

	for(ii=0; ii<2; ii++){	//0:CN101(HLS)��, 1:CN99(�����E���i�p�t�@�[��)��
		if( ii == 0 ){
			//���i�pCPLD�������ݎ��ASP_MOTHER_BOARD�̌�������
			//CN101(HLS CPLD��)�̏������݂͕K�v�Ȃ�
			if( (nCpldId != 0) ||
                (m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD && m_nIniSpMotherBoardType ==  0) ){
				continue;
			}
		}
		
		if( nCpldId == 0 ){	//�����pCPLD�������ݎ�
			printf("\n");
			printf("------------------------------------------------------\n");
			if( ii == 0 ){
				printf("  ��CN101(HLS CPLD��)�̏������݁�\n");
			}
			else{
				printf("  ��CN99(�����pCPLD)���̏������݁�\n");
			}
		}
		else{				//���i�pCPLD�������ݎ�
			printf("\n");
			printf("------------------------------------------------------\n");
			printf("  ��CN99(���i�pCPLD)���̏������݁�\n");
		}

		printf("   Quartus II ���g�p����CPLD�̏������݂��J�n���Ă��������B\n");
		printf("------------------------------------------------------\n");
		printf("\n");
	
		//CPLD�������ݗpJTAG�ڑ��̐؂�ւ�
		switch(ii){
		case 0:
			SnRelayOutputBit(BIT_SEL_CPLDSEL_CTRL, CPLD_CONNECTOR_SEL_CN101);
			break;
		case 1:
			SnRelayOutputBit(BIT_SEL_CPLDSEL_CTRL, CPLD_CONNECTOR_SEL_CN99);
			break;
		}

		printf("�I������ꍇ�́A'Y'(����I��)�܂��́A'N'(�ُ�I��)����͂��Ă�������\n");
		while(ForeverLoop){
			nInputVal = KeyInputWait();
			if( nInputVal == 'y' || nInputVal == 'Y' || nInputVal == 0x0d ){
				//����I��
				break;
			}
			else if( nInputVal == 'n' || nInputVal == 'N' || nInputVal == 0x1B ){
				//�ُ�I��
				bRet = FALSE;
				break;
			}
			else{	//���̑��̃L�[�͖�������
				continue;
			}
		}	

		if( nCpldId == 0 ){	//�����pCPLD�������ݎ�
			if( ii == 0 ){
				LogPrintfCsv(m_strLogFileName,"  [%s]: CN101(HLS CPLD)���̏�������\n", ((bRet==TRUE) ? "OK":"NG"));
			}
			else{
				LogPrintfCsv(m_strLogFileName,"  [%s]: CN99(�����pCPLD)���̏�������\n", ((bRet==TRUE) ? "OK":"NG"));
			}
		}
		else{				//���i�pCPLD�������ݎ�
			LogPrintfCsv(m_strLogFileName,"  [%s]: CN99(���i�pCPLD)���̏�������\n", ((bRet==TRUE) ? "OK":"NG"));
		}
	}

	/////////////////////////////////////////////////////////////////////
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"CPLD�������݁F%s\n", ((bRet==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	/////////////////////////////////////////////////////////////////////
	//�I������
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:CPLD�������� %s %s FINISH ������\n", nInsID, strDate, strTime);

	return bRet;
}

//////////////////////////////////////////
//SP_XXXX��̎�ʐݒ���s��
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
		"\n������ %c:Board ID Check %s %s START ������\n", nInsID, strDate, strTime);

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
			LogPrintfCsv(m_strLogFileName, "�{�[�h���Őݒ肳��Ă���^�C�v��[%s]�ł��B\n", m_cSPxxxxBoardName);

			if( RS_SpSetAdOffsetGain(dwAdOffsetTemp, dwAdGainTemp, dwAdOffsetTemp, dwAdGainTemp) == TRUE ){	//AD�␳�l�̓Ǐo���̂ݎ��s
				if( RS_SpSetDaOffsetGain(dwDaOffsetTemp, dwDaGainTemp, dwDaOffsetTemp, dwDaGainTemp) == TRUE ){	//DA�␳�l�̓Ǐo���̂ݎ��s
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
					printf("D/A Offset/Gain�ݒ�l���m�F�ł��܂���ł����B\n");
				}
			}
			else{
				printf("A/D Offset/Gain�ݒ�l���m�F�ł��܂���ł����B\n");
			}
		}
		else{
			LogPrintfCsv(m_strLogFileName, "�{�[�h���Őݒ肳��Ă���^�C�v���m�F�ł��܂���ł����B\n");
		}

		//�{�[�h���̐ݒ��INI�̐ݒ肪�قȂ��Ă��邩���m�F����
		bMatchChk = FALSE;
		if( m_nSPxxxxBoardID != -1 ){
			switch(m_nSPxxxxBoardID){
			case SP_BID_UPPER_IF_BOARD:
				if( m_nIniBoardID == 0 ){
					bMatchChk = TRUE;	//�{�[�h���̐ݒ��INI�̐ݒ�͓���
				}
				break;
			case SP_BID_FRONT_IF_BOARD:
				if( m_nIniBoardID == 1 ){
					bMatchChk = TRUE;	//�{�[�h���̐ݒ��INI�̐ݒ�͓���
				}
				break;
			case SP_BID_MOTHER_BOARD:
				if( m_nIniBoardID == 2 ){
					bMatchChk = TRUE;	//�{�[�h���̐ݒ��INI�̐ݒ�͓���
				}
				break;
			default:
				break;
			}
		}

		nNewBoardID = -1;
		//�{�[�h���̐ݒ��INI�̐ݒ肪�قȂ��Ă���Ƃ��iINI�̐ݒ�l���{�[�h�ɐݒ肷�邩���m�F����j
		if( bMatchChk != TRUE ){
			switch(m_nIniBoardID){
			case 0:
				printf("�{�[�h�^�C�v(Upper,Front,Mother)��\"Upper\"�ɐݒ肵�܂����H(y or n) -->" );
				break;
			case 1:
				printf("�{�[�h�^�C�v(Upper,Front,Mother)��\"Front\"�ɐݒ肵�܂����H(y or n) -->" );
				break;
			case 2:
				printf("�{�[�h�^�C�v(Upper,Front,Mother)��\"Mother\"�ɐݒ肵�܂����H(y or n) -->" );
				break;
			default:
				bMatchChk = TRUE;
				break;
			}

			if( bMatchChk != TRUE ){
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					//INI�̐ݒ�l���{�[�h�ɐݒ肷��
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

		//�{�[�hID�̕ύX���s���Ƃ��iUpper�AFront�AMother��I�����Ă��炤�j
		if( nNewBoardID == -1 ){
			printf("�{�[�h�^�C�v�̐ݒ��ύX���܂����H(y or n) -->" );
			nInputVal = KeyInputWaitYorN();
			if( nInputVal == 'y' ){
				while(ForeverLoop){
					printf("\n�{�[�h��I�����Ă�������[0:Upper, 1:Front, 2:Mother] -->");
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

		//�{�[�hID���{�[�h���ɑ��M��EEPROM�ɋL�^����
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
				LogPrintfCsv(m_strLogFileName, "�{�[�hID��(%s)�ɐݒ肵�܂����B\n", m_cSPxxxxBoardName );
				bRet = TRUE;
			}
			else{
				printf("�{�[�hID�̐ݒ�Ɏ��s���܂����B\a\n" );
			}
		}
	}

	/////////////////////////////////////////////////////////////////////
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"Board ID Check�F%s\n", ((bRet==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:Board ID Check %s %s FINISH ������\n", nInsID, strDate, strTime);

	return TRUE;
}

///////////////////////////////////////////////////////
//�ʐM����(SP��p)�@nComID=0(NET1��):1(NET2��)
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
		"������ %c:HLS�ʐM�`�F�b�N(NET%d) %s %s START ������\n", nInsID, nComID+1, strDate, strTime);
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
	//	//DSW�ݒ�������p�̃A�h���X�ŏI������ꍇ
	//	nTestPatNum--;
	//}

	for( ii=0; ii<nTestPatNum; ii++ ){
		if( ii == 0 ){
			if( nComID == 1 ){	//NET1��NET2�̌����͍ŏ��ƍŌ��2����{����悤�ɂ���
				//ii = (nTestPatNum-2);	//�����I��NET2���̌����Ɉڍs����
				ii = 2;	//�����I��NET2���̌����Ɉڍs����
			}
			else{
				printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				printf("NET1�R�l�N�^����HLS�ʐM�P�[�u����ڑ����Ă�������\n");
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
			if( nComID == 0 ) break;	//NET1��NET2�̌����͍ŏ��ƍŌ��2����{����悤�ɂ���
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			printf("NET2�R�l�N�^����HLS�ʐM�P�[�u����ڑ����Ă�������\n");
			printf("�������ł����牽���L�[�������Ă��������B\n");
			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			nInputVal = KeyInputWait();
			if ( nInputVal == 0x1B ){
				bRetCont = FALSE;
				break;
			}
		}
		bSwSetReqDisp = FALSE;
		for( kk=0; kk<3; kk++ ){	//DSW���������Z�b�g���ꂽ�����i�葀��̂��߁j�R��`�F�b�N����
			while(bRetCont == TRUE){
				bComStat = TRUE;	//�w��͈̓A�h���X�̒ʐM�E��ʐM��ԃ`�F�b�N����
				for(jj=0; jj<8; jj++){
					if( (pTestPat[ii].nComStartAdr[jj] != -1) && (pTestPat[ii].nComEndAdr[jj] != -1) ){
						if( pTestPat[ii].nSwTestCode[jj] <= 0 ){
							bComChk = FALSE;	//��ʐM��Ԃł��邱�Ƃ��`�F�b�N����
						}
						else{
							bComChk = TRUE;		//�ʐM��Ԃł��邱�Ƃ��`�F�b�N����
						}
						if( SpComChk(pTestPat[ii].nComStartAdr[jj], pTestPat[ii].nComEndAdr[jj], bComChk) != TRUE ){
							bComStat = FALSE;
						}
					}
				}
				if( bComStat == TRUE ){
					break;	//�SSW���������Z�b�g���ꂽ���Ƃ����o
				}
				else if(bSwSetReqDisp == FALSE){
					bSwSetReqDisp = TRUE;
					printf("\nHLS�ʐM�e�X�g[%d/%d]\n", ii+1, nTestPatNum);

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
							if( jj==2 ){	//Upper��SW3���̓e�X�g
								if(pTestPat[ii].nSwTestCode[jj] == 0){
									printf("SW%d:%2d(0,0,0,0,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
									//printf("SW%d��%d(�SOFF)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								}
								else if( pTestPat[ii].nSwTestCode[jj] == 12 ){
									printf("SW%d:%2d(0,0,1,1,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
									//printf("SW%d��%d(1:OFF, 2:OFF, 3:ON, 4:ON, 5:OFF, 6:OFF)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								}
								else if( pTestPat[ii].nSwTestCode[jj] == 21 ){
									printf("SW%d:%2d(1,0,1,0,1,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
									//printf("SW%d��%d(1:ON, 2:OFF, 3:ON, 4:OFF, 5:ON, 6:OFF)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								}
								else if( pTestPat[ii].nSwTestCode[jj] == 42 ){
									printf("SW%d:%2d(0,1,0,1,0,1)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
									//printf("SW%d��%d(1:OFF, 2:ON, 3:OFF, 4:ON, 5:OFF, 6:ON)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								}
								else{
									printf("SW%d��%2d(�J�nAdr=%d, �I��Adr=%d)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj],
											pTestPat[ii].nComStartAdr[jj], pTestPat[ii].nComEndAdr[jj]);
								}
							}
							else if(pTestPat[ii].nSwTestCode[jj] == 0){
								printf("SW%d:%2d(0,0,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%d��%d(�SOFF)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else if( pTestPat[ii].nSwTestCode[jj] == 5 ){
								printf("SW%d:%2d(1,0,1,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%d��%d(1:ON, 2:OFF, 3:ON, 4:OFF)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else if( pTestPat[ii].nSwTestCode[jj] == 10 ){
								printf("SW%d:%2d(0,1,0,1)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%d��%d(1:OFF, 2:ON, 3:OFF, 4:ON)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else if( pTestPat[ii].nSwTestCode[jj] == 1 ){
								printf("SW%d:%2d(1,0,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%d��%d(1:ON, 2:OFF, 3:OFF, 4:OFF)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else if( pTestPat[ii].nSwTestCode[jj] == 2 ){
								printf("SW%d:%2d(0,1,0,0)\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
								//printf("SW%d��%d(1:OFF, 2:ON, 3:OFF, 4:OFF)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj]);
							}
							else{
								printf("SW%d��%2d(�J�nAdr=%d, �I��Adr=%d)�ɐݒ肵�Ă�������\n", jj+1, pTestPat[ii].nSwTestCode[jj],
										pTestPat[ii].nComStartAdr[jj], pTestPat[ii].nComEndAdr[jj]);
							}

						}
					}
					printf("HLS�A�h���X�ݒ�pDSW%d�`DSW%d����L�̐ݒ�ɂ��Ă��������B\n", nDswMin+1, nDswMax+1);
					printf("(Esc�L�[:�e�X�gCancel)\n");
				}
				if( _kbhit() ){
					nInputVal = _getch();
					if( nInputVal == 0x1B ){
						bRetCont = FALSE;
						printf("\a�ʐM�e�X�g���L�����Z������܂���\a\n");
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
				bComStat = TRUE;	//�w��͈̓A�h���X�̒ʐM�E��ʐM��ԃ`�F�b�N����
				for(jj=0; jj<8; jj++){
					if( pTestPat[ii].nSwTestCode[jj] != -1 ){
						if( pTestPat[ii].nSwTestCode[jj] == 0 ){
							bComChk = FALSE;	//��ʐM��Ԃł��邱�Ƃ��`�F�b�N����
						}
						else{
							bComChk = TRUE;		//�ʐM��Ԃł��邱�Ƃ��`�F�b�N����
						}
						if( SpComChk(pTestPat[ii].nComStartAdr[jj], pTestPat[ii].nComEndAdr[jj], bComChk) != TRUE ){
							printf("\n");
							if( bComChk == TRUE ){
								LogPrintfCsv(m_strLogFileName,
											"[HLS�ʐM����] SW%d, Adr%d �ʐM�`�F�b�N�G���[\n", 
											jj+1, pTestPat[ii].nComStartAdr[jj]);
							}
							else{
								LogPrintfCsv(m_strLogFileName,
											"[HLS�ʐM����] SW%d, Adr%d ��ʐM�`�F�b�N�G���[\n", 
											jj+1, pTestPat[ii].nComStartAdr[jj]);
							}
							printf("�����L�[�������Ă��������B\a\n");
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
					//�o�ߕ\��
					if((DWORD)(GetTickCount()-dwStartTick) > (DWORD)(pTestPat[ii].dwChkDurMsec-((pTestPat[ii].dwChkDurMsec/4) * 1))){
						printf("\r����\r");
					}
					else if((DWORD)(GetTickCount()-dwStartTick) > (DWORD)(pTestPat[ii].dwChkDurMsec-((pTestPat[ii].dwChkDurMsec/4) * 2))){
						printf("\r���\r");
					}
					else if((DWORD)(GetTickCount()-dwStartTick) > (DWORD)(pTestPat[ii].dwChkDurMsec-((pTestPat[ii].dwChkDurMsec/4) * 3))){
						printf("\r��\r");
					}
					else if((DWORD)(GetTickCount()-dwStartTick) > 0){
						printf("\r�\r");
					}
				}
			}while((GetTickCount()-dwStartTick) < pTestPat[ii].dwChkDurMsec);
		}
		if( bRetCont != TRUE )break;
	}

	printf("\n");

	/////////////////////////////////////////////////////////////////////
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"HLS�ʐM�`�F�b�N�F%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:HLS�ʐM�`�F�b�N(NET%d) %s %s FINISH ������\n", nInsID, nComID+1, strDate, strTime);
	
	return bRetCont;
}

///////////////////////////////////////////////////////
//DIO�܂�Ԃ�����
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
		"������ %c:%s DIO����(AUTO) %s %s START ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);

	//�������SP_UPPER1 or 2�̂Ƃ�
	if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
		if( bManualTest == TRUE ){
			if( m_nIniSpUpperBoardType == 1 ){	//Upper2?
				berr = SPxxxxDioChkManualUpper2();	//Upper2 I/O�����i�蓮�j
			}
			else{
				berr = SPxxxxDioChkManualUpper1();	//Upper1 I/O�����i�蓮�j
			}
			if( berr != TRUE ){
				bRetCont = berr;
				printf("���̂܂܌������p�����܂����H(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal == 'y' ){
					berr = TRUE;	//�������Ɉڍs����
				}
			}
		}
		if( berr == TRUE && bAutoTest == TRUE){
			if( m_nIniSpUpperBoardType == 1 ){	//Upper2
				bRetCont = SPxxxxDioChkAutoUpper2();	//Upper2 I/O�����i�����j
			}
			else{
				bRetCont = SPxxxxDioChkAutoUpper1();	//Upper1 I/O�����i�����j
			}
		}
	}
	//�������SP_FRONT2�̂Ƃ�
	else if( m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ){
		bRetCont = SPxxxxDioChkAutoFront2();
	}
	//�������SPIN�܂���SP_MOTHER�̂Ƃ�
	else if( SP_BID_MOTHER_BOARD ){
		if( m_nIniSpMotherBoardType != 0 ){	//SPIN_MOTHER_BOARD
			bRetCont = SPxxxxDioChkAutoSpinMother();
		}
		else{	//SP_MOTHER_BOARD
		}
	}

	/////////////////////////////////////////////////////////////////////
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"DIO����(AUTO)�F%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	/////////////////////////////////////////////////////////////////////
	//DIO�����I������
	RS_SpNormalMode();
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:%s DIO����(AUTO) %s %s FINISH ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

////////////////////////////////////////////
//SPIN_MOTHER_BOARD SAVENET�𗘗p����DIO����
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
	//HLS�ʐM��~���̏o��OFF�e�X�g
	/////////////////////////////////////////////////////////////////////
	printf("\n");
	printf("-----------------------------------------------\n");
	printf(" HLS�ʐM��~���̏o��OFF�m�F\n");
	printf("-----------------------------------------------\n");
	memset(wTestPatten, 0, sizeof(wTestPatten));
	//CN61 OUT1_0�`OUT1_f �o��OFF�e�X�g
	wTestPatten[nPatternCnt++] = TEST_PATTERN10;
	//CN33 OUT2_0�`OUT2_3 �o��OFF�e�X�g
	wTestPatten[nPatternCnt++] = TEST_PATTERN15;
	//CN41 OUT3_0�`OUT3_b �o��OFF�e�X�g
	wTestPatten[nPatternCnt++] = TEST_PATTERN16;
	//CN42 OUT3_0�`OUT3_b �o��OFF�e�X�g
	wTestPatten[nPatternCnt++] = TEST_PATTERN18;

	while(1){
		if( HlsOutStopTest(m_wSpinTestPattern, wTestPatten, nPatternCnt, 300) != TRUE ){
			printf("\n�����𒆎~���܂����H(y or n) -->");
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
	//DI���� (1�_�P�ʂ̓��͌���)
	/////////////////////////////////////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n\n");
		printf("-----------------------------------------------\n");
		printf("SPIN_MOTHER_BOARD: DIO�������J�n���܂����B\n");
		for( ii=TEST_PATTERN0; ii<TEST_PATTERN0+SPIN_TEST_PATTERN_NUM; ii++ ){
			/*
			if( ii == TEST_PATTERN2 || ii == TEST_PATTERN8 || ii == TEST_PATTERN9 || ii == TEST_PATTERN15 || ii == TEST_PATTERN16){
				bZeroChkEnable = FALSE;	//�p�^�[��2, 8, 9, 15, 16�ł͕���ON���͂����邽�ߑ�Adr��Zero�`�F�b�N�͂��Ȃ�
			}
			else{
				bZeroChkEnable = TRUE;
			}
			*/
			bZeroChkEnable = TRUE;		//�Ƃ肠�����S�Ẵe�X�g�p�^�[���ő�Adr��Zero�`�F�b�N�����{���Ă���

			if( m_wSpinTestPattern[ii].wInAdr >= SP_XXXX_SW2_START_ADR+1 ){
				wAnalogBitMask = 0xc000;	//�A�i���O���̓A�h���X�͉���14bit��0�Ń}�X�N����
			}
			else{
				wAnalogBitMask = DIO_TEST_ALL_BIT;
			}
			
			if( bRetCont == TRUE ){
				bRetCont = DioAutoTestSelPattern(m_wSpinTestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);
				if( bRetCont != TRUE ){
					printf("\n�����𒆎~���܂����H(y or n) -->");
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						break;
					}
				}
			}
			else{
				DioAutoTestSelPattern(m_wSpinTestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);	//NG���������������������𑱍s
				//break;
			}
		}
	}
	return bRetCont;
}

///////////////////////////////////////////////////////
//�f�W�^�����o�͕����e�X�g(SPIN_MOTHER_BOARD)
enum{
	S1_SW=1,
	S2_SW,
	S3_SW,
	S4_SW,
};
#define READCHK_LOOP_NUM		32		//�ǂݏo���J��Ԃ���
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
		"������ %c:%s DIO����(OTHER) %s %s START ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);


	//���͑҂��f�[�^�G���A�𖳌��l�ŏ���������
	for(ii=0;ii<64;ii++){
		lnDiWaitVal[ii] = -1;
	}
	///////////////////////////////////////////////////
	//SW����ɂ����� (S4-4�Ԃ�ON�̂Ƃ���)�m�F
	printf("\n");
	printf("-------------------------------------------------\n");
	printf("   S4�X�C�b�`��4�Ԃ݂̂�ON�ɐݒ肵�Ă��������B\n");
	printf("   S1�`S3�X�C�b�`�͑S��OFF�ɐݒ肵�Ă��������B\n");
	printf("-------------------------------------------------\n");
	printf("\n");

	//IN2-f, IN3-f��ON�ɂȂ�̂�҂�
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x8000;	//IN2
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x8000;	//IN3
	bRetCont = SnDiReadWait(lnDiWaitVal, 500);

	////////////////////////////
	//S1-1�`8�X�C�b�`���͊m�F
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S1_SW, 1, (WORD)(SP_XXXX_SW1_START_ADR+0), 0, 8);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S1-1�`8�X�C�b�`���̓e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}
	////////////////////////////
	//S2-1�`8�X�C�b�`���͊m�F
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S2_SW, 1, (WORD)(SP_XXXX_SW1_START_ADR+1), 0, 8);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S2-1�`8�X�C�b�`���̓e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}
	/////////////////////////////////////
	//OUT1_0�`3 �� IN3_7�`a���͊m�F(�����͎���)
	if( bRetCont == TRUE ){
		printf("\n");
		printf("---------------------------------------------\n");
		printf("OUT1_0�`3 �� IN3_7�`a �܂�Ԃ����̓e�X�g��...\n");
		for( ii=0; ii<4; ii++ ){
			DATAout(BoardId, (WORD)(SP_XXXX_SW1_START_ADR+0), (WORD)(0x0001<<ii));

			//IN3-7�`IN3-a������ON�ɂȂ�̂�҂�
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x8000;	//IN2
			lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x8000 | (0x0080 << ii);	//IN3
			bRetCont = SnDiReadWait(lnDiWaitVal, 100);
			if( bRetCont == TRUE ){
				//IN3-7�`IN3-a��OFF�ɂȂ�̂�҂�
				lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
				lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x8000;	//IN2
				lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x8000;	//IN3
				bRetCont = SnDiReadWait(lnDiWaitVal, 100);
			}		
			if( bRetCont != TRUE ){
				break;
			}
		}
		LogPrintfCsv(m_strLogFileName, "OUT1_0�`3 �� IN3_7�`a �܂�Ԃ����̓e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}	

	////////////////////////////
	//S3-1�`4�X�C�b�`���͊m�F
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S3_SW, 1, (WORD)(SP_XXXX_SW1_START_ADR+2), 0xb, 4);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S3-1�`4�X�C�b�`���̓e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	////////////////////////////
	//S4-3�X�C�b�`���͊m�F
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S4_SW, 3, (WORD)(SP_XXXX_SW1_START_ADR+1), 0xe, 1);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S4-3�X�C�b�`���̓e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	////////////////////////////
	//S4-4�Ԃ�OFF�ɖ߂�
	printf("\n");
	printf("-------------------------------------------------\n");
	printf("   S4�X�C�b�`��4�Ԃ�OFF�ɖ߂��Ă��������B\n");
	printf("-------------------------------------------------\n");
	printf("\n");

	//IN2-f, IN3-f��OFF�ɂȂ�̂�҂�
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x0000;	//IN2
	lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x0000;	//IN3
	bRetCont = SnDiReadWait(lnDiWaitVal, 500);

	///////////////////////////////////////////////////
	//SW����ɂ����� (S4-4�Ԃ�OFF�̂Ƃ���)�m�F
	////////////////////////////
	//S4-1�`2�X�C�b�`���͊m�F
	if( bRetCont == TRUE ){
		bRetCont = SpMotherSxSwTest(S4_SW, 1, (WORD)(SP_XXXX_SW1_START_ADR+1), 0xe, 2);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S4-1�`2�X�C�b�`���̓e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	//CN14(#51-B4:OFF(24V), #51-B5:ON(G))��IN3_e ����ON�m�F(Auto)
	if( bRetCont == TRUE ){
		printf("\n");
		printf("---------------------------------------------\n");
		printf("CN14(SIG_24V,GND)���� �� IN3_4�܂�Ԃ����̓e�X�g��...\n");

		DATAout(BoardId, ADR_DO_51, 0x0030);	//(#51) B4:ON(SIG-24V), B5:ON(SIG-GND)
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x0000;	//IN2
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x0010;	//IN3 (B4:ON)
		bRetCont = SnDiReadWait(lnDiWaitVal, 100);
		//�ȉ��͏���������Ȃ��Ƃ���IN3-B4�̓��͂�OFF�ɂȂ邱�Ƃ̊m�F
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

		LogPrintfCsv(m_strLogFileName, "CN14(SIG_24,GND)���� �� IN3_4�܂�Ԃ����̓e�X�g%s\n",
					((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	//�ȉ������p�^�[���̌���(nMotor_Stop�Ȃǥ��)
	///////////////////////////////////////
	//nFFU Stop(S1-1)�e�X�g
	//CN12(19_FFU_Stop_a):#53-B0 �� CN12(20_FFU_Stop_b):#58-B0 ���܂�Ԃ����Ƃ��m�F����
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ��nFFU_Stop�e�X�g��\n");
		printf("   S1�X�C�b�`��1�Ԃ݂̂�ON�ɐݒ肵�Ă��������B\n");
		printf("------------------------------------------------------\n");
		bRetCont = DioLoopBackTest(ADR_DO_53, 0, ADR_DI_58, 0);
		LogPrintfCsv(m_strLogFileName, "(S1-1):nFFU_Stop�e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("------------------------------------------------------\n");
	}

	///////////////////////////////////////
	//nCamber_SV_Stop(S1-2)�e�X�g
	//LED:[B-CN23]���5(��CN22-2), 6(��CN21-1), 7(��CN21-2)�̓_��/������ڎ��m�F����
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ��nCamber_SV_Stop��\n");
		printf("   S1�X�C�b�`��2�Ԃ݂̂�ON�ɂ��邱�ƂŁA\n");
		printf("   �����@�O�ʃp�l��[B-CN23]��LED5�`LED7��ON���邱�Ƃ��m�F���Ă��������B\n");
		printf("------------------------------------------------------\n");
		printf("   [B-CN23]LED5�`LED7�͐���ɓ_�����Ă��܂����H(y or n) -->");
		nInputVal = KeyInputWaitYorN();
		if( nInputVal == 0x1B ){
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED5�`LED7�ڎ��m�F�𒆒f���܂���\n");
			bRetCont = FALSE;		//�������f
		}
		else if( nInputVal == 'y' ){
			//LED �_���E�����ڎ�����OK
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED5�`LED7�ڎ��m�FOK\n");
		}
		else{
			//LED �_���E�����ڎ�����NG
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED5�`LED7�ڎ��m�FNG\n");
			bRetCont = FALSE;		//�d���l�ُ�
		}
	}

	///////////////////////////////////////
	//nMotor_Stop(S1-3)�e�X�g
	//CN13 #52-B9 �� IN5-A9
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ��nFFU_Stop�e�X�g��\n");
		printf("   S1�X�C�b�`��3�Ԃ݂̂�ON�ɐݒ肵�Ă��������B\n");
		printf("------------------------------------------------------\n");
		bRetCont = DioLoopBackTest(ADR_DO_52, 9, (WORD)(SP_XXXX_SW1_START_ADR+1), 9);
		LogPrintfCsv(m_strLogFileName, "(S1-3):nMotor_Stop�e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("------------------------------------------------------\n");
	}

	///////////////////////////////////////
	// 24V_IL(S1-4)�A(S1-6)�e�X�g
	// S1-4,S1-6 ON �� [B-CN23] 1�`4 LED�ڎ��m�F
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ��24V_IL��\n");
		printf("   S1�X�C�b�`��4�Ԃ�6�Ԃ݂̂�ON�ɂ��邱�ƂŁA\n");
		printf("   �����@�O�ʃp�l��[B-CN23]��LED1�`LED4��ON���邱�Ƃ��m�F���Ă��������B\n");
		printf("------------------------------------------------------\n");
		printf("   [B-CN23]LED1�`LED4�͐���ɓ_�����Ă��܂����H(y or n) -->");
		nInputVal = KeyInputWaitYorN();
		if( nInputVal == 0x1B ){
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED1�`LED4�ڎ��m�F�𒆒f���܂���\n");
			bRetCont = FALSE;		//�������f
		}
		else if( nInputVal == 'y' ){
			//LED �_���E�����ڎ�����OK
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED1�`LED4�ڎ��m�FOK\n");
		}
		else{
			//LED �_���E�����ڎ�����NG
			LogPrintfCsv(m_strLogFileName, "(S1-2):[B-CN23] LED1�`LED4�ڎ��m�FNG\n");
			bRetCont = FALSE;		//�d���l�ُ�
		}
		printf("------------------------------------------------------\n");
	}
	
	///////////////////////////////////////
	//nTower_SV_Stop(S1-5)�e�X�g
	// #53-B2 �� #58-A2�A#51-B7 �� #3-bit6
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   ��nTower_SV_Stop�e�X�g��\n");
		printf("   S1�X�C�b�`��5�Ԃ݂̂�ON�ɐݒ肵�Ă��������B\n");
		printf("------------------------------------------------------\n");

		bRetCont = DioLoopBackTest(ADR_DO_53, 2, ADR_DI_58, 2);
		LogPrintfCsv(m_strLogFileName, "(S1-5):Tower IL b�e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));

		berr = DioLoopBackTest(ADR_DO_51, 7, (WORD)(SP_XXXX_SW1_START_ADR+1), 6);
		LogPrintfCsv(m_strLogFileName, "(S1-5):Tower IL IN�e�X�g%s\n", ((berr == TRUE) ? "OK" : "NG"));
		if( berr != TRUE ){
			bRetCont = FALSE;
		}

		printf("------------------------------------------------------\n");
	}

	///////////////////////////////////////
	//IPA_Dispense_IL(S2-1)�e�X�g
	// S2-1 �� #56-A2
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x8000);	//OUT2-F ON
		bRetCont = SpMotherSxSwTest(S2_SW, 1, ADR_DI_56, 2, 1);
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x0000);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S2-1�X�C�b�`���̓e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}
	
	///////////////////////////////////////
	//Reserve_Dispense_IL(S2-2)�e�X�g
	// S2-2 �� #56-A1
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x4000);	//OUT2-E ON
		bRetCont = SpMotherSxSwTest(S2_SW, 2, ADR_DI_56, 1, 1);
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x0000);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S2-2�X�C�b�`���̓e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	///////////////////////////////////////
	//H2O4_Dispense_IL(S2-3)�e�X�g
	// S2-3 �� #56-A0
	///////////////////////////////////////
	if( bRetCont == TRUE ){
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x2000);	//OUT2-D ON
		bRetCont = SpMotherSxSwTest(S2_SW, 3, ADR_DI_56, 0, 1);
		DATAout(BoardId, SP_XXXX_SW1_START_ADR+1, 0x0000);
		printf("\n");
		printf("---------------------------------------------\n");
		LogPrintfCsv(m_strLogFileName, "S2-3�X�C�b�`���̓e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}

	//////////////////////////////////////
	// CN13-9,CN13-11,CN31-15 �� CN31-1,CN33-B1 AND��H�e�X�g
	if( bRetCont == TRUE ){
		//
		//#52-B8(IL_IG_SW_a) ,#52-B9(EMG_1a) ��#58-A3(EMG) ���̓e�X�g
		//
	
		//����������Ȃ��Ƃ���#58-A3,A4�̓��͂�OFF�ɂȂ邱�Ƃ̊m�F
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

		//�������������Ƃ���#58-A3�̓��͂�ON�ɂȂ邱�Ƃ̊m�F
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

		//#58-A3��ON���邱�Ƃ��m�F����
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
		LogPrintfCsv(m_strLogFileName, "CN13-9,CN13-11,CN31-15 �� CN31-1,CN33-B1 AND��H�e�X�g%s\n", ((bRetCont == TRUE) ? "OK" : "NG"));
		printf("---------------------------------------------\n");
	}
	
	/////////////////////////////////////////////////////////////////////
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"DIO����(OTHER)�F%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	/////////////////////////////////////////////////////////////////////
	//DIO�����I������
	RS_SpNormalMode();
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:%s DIO����(OTHER) %s %s FINISH ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}
//////////////////////////////////////////////////////
//  SP_MOTHER, SPIN_MOTHER�p S1�`S4�X�C�b�`���͌���
BOOL SpMotherSxSwTest(int nSwNo, int nSwStart, WORD wInAdr, WORD wInStartBit, int nBitCount)
{
	long	lnDiWaitVal[64];
	BOOL	bRet=TRUE;
	int		ii;

	//���͑҂��f�[�^�G���A�𖳌��l�ŏ���������
	for(ii=0;ii<64;ii++){
		lnDiWaitVal[ii] = -1;
	}

	for( ii=0; ii<nBitCount; ii++ ){
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("   S%d�X�C�b�`��%d�Ԃ݂̂�ON�ɐݒ肵�Ă��������B\n", nSwNo, nSwStart+ii);
		printf("------------------------------------------------------\n");
		printf("\n");

		//IN1�`IN3�̊Y��bit�݂̂�ON�ɂȂ�̂�҂�
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+0] = 0x0000;	//IN1
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+1] = 0x8000;	//IN2 (bit15�͏펞ON)
		lnDiWaitVal[SP_XXXX_SW1_START_ADR+2] = 0x8000;	//IN3 (bit15�͏펞ON)
		lnDiWaitVal[wInAdr] = lnDiWaitVal[wInAdr] | (0x0001 << (wInStartBit+ii));
		bRet = SnDiReadWait(lnDiWaitVal, 500);
		if( bRet == TRUE ){
			printf("\n");
			printf("------------------------------------------------------\n");
			printf("   S%d�X�C�b�`��%d�Ԃ�OFF�ɖ߂��Ă��������B\n", nSwNo, nSwStart+ii);
			printf("------------------------------------------------------\n");
			printf("\n");

			//IN1�`IN3��OFF�ɂȂ�̂�҂�
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
//���͒l�����肵�ēǂݏo����悤�ɂȂ�܂ő҂�
#define	DI_READ_CHK_COUNT			(30)
BOOL	SnDiReadWait(long *plnDiWaitVal, DWORD dwFirstWaitMsec)	//[64]�̂����z��[0]�͎g��Ȃ�
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
	printf("\n���͒l�m�F�����[1/%d]\n", DI_READ_CHK_COUNT);

	while( bRet == TRUE ){
		if( _kbhit() ){
			nInputVal = _getch();
			if( nInputVal == 0x1B ){
				bRet = FALSE;
				LogPrintfCsv(m_strLogFileName, "[NG] ���͑҂����L�����Z�����܂���\n");
				printf("\a");
				break;
			}
		}

		bInputChk = TRUE;
		for(wChkAdr=1; wChkAdr<=63; wChkAdr++){
			//���͑҂��f�[�^�͗L���ȃf�[�^���H
			if( plnDiWaitVal[wChkAdr] >= 0x0000 && plnDiWaitVal[wChkAdr] <= 0xffff ){
				DATAin(BoardId, wChkAdr, (LPINT*)&dwInData);
				//////////////////////////////////////////////////
				//�A�i���O���̓A�h���X�Ȃ�΁A����12bit��0�Ń}�X�N����
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
				//���̓f�[�^����v���邩���m�F����
				if( (DWORD)plnDiWaitVal[wChkAdr] != dwInData ){
					//���̓f�[�^�s��v
					bInputChk = FALSE;
					if( nTestCountNum > 0 ){
						printf("\r[NG] Adr=%d, WAIT=%04lX, DiVal=%04X [%d/%d] \n",
							wChkAdr, plnDiWaitVal[wChkAdr], dwInData,
							nTestCountNum, DI_READ_CHK_COUNT);
					}
					break;
				}
				else{
					//���̓f�[�^OK
					printf("\r[OK] Adr=%d, WAIT=%04lX, DiVal=%04X [%d/%d] \r",
						wChkAdr, plnDiWaitVal[wChkAdr], dwInData,
						nTestCountNum, DI_READ_CHK_COUNT);
				}
			}
		}
		
		if( bRet == TRUE && bInputChk == TRUE ){
			if( nTestCountNum == 0 ){	//�����v�m�F��́A���̓`���^�����O�h�~�p��WAIT�����΂�
				Sleep_Cnt(dwFirstWaitMsec);
			}
			nTestCountNum++;
			//DI_READ_CHK_COUNT��A�����ăf�[�^���ǂݏo�����OK�Ƃ���
			if( nTestCountNum >= DI_READ_CHK_COUNT ){
				break;		//����OK
			}
			printf("\n���͒l�m�F�����[%d/%d]\n", nTestCountNum+1, DI_READ_CHK_COUNT);
		}
		else{
			if( nTestCountNum > 0 ){
				LogPrintfCsv(m_strLogFileName, "\n[���͒l���s����ł�] Adr=%d, WAIT=%04lX, DiVal=%04X\n",
							wChkAdr, plnDiWaitVal[wChkAdr], dwInData);
				nTestCountNum = 0;
				bRet = FALSE;
				break;
			}
		}
		Sleep_Cnt(10);
	}	//while end (���͑҂�)

	return bRet;
}

///////////////////////////////////////////////////////
//A/D���͐��x�m�F(SP_XXXX_BOARD�p)
#define	AD_INPUT_RANGE_4_20mA		(0)
#define	AD_INPUT_RANGE_0_5V			(1)
#define	AD_INPUT_RANGE_0_10V		(2)
#define	AD_INPUT_RANGE_NUM			(AD_INPUT_RANGE_0_10V+1)
#define	AD_INPUT_RANGE_CHK_NUM		(5)
double	m_fAdInputStdVal[AD_INPUT_RANGE_NUM][AD_INPUT_RANGE_CHK_NUM]={
	//MIN,  MID,  MAX, OVR(L), OVR{H)
	{ 4.0, 12.0, 20.0,   3.96,  20.04},		//4-20mA�����W���̊m�F�p�̒l
	{ 0.0,  2.5,  5.0, -0.013,  5.013},		//0-5V�����W���̊m�F�p�̒l
	{ 0.0,  5.0, 10.0, -0.025, 10.025}		//0-10V�����W���̊m�F�p�̒l
};

// nInsID  : ����ID�ԍ�
// nSch    : A/D�ϊ��J�nch(-1�w�莞��Default��0�Ƃ���)
// nEch    : A/D�ϊ��I��ch(-1�w�莞��SP����Ɋ��蓖�Ă���ő�ch�ԍ��Ƃ���)
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
	int		nIinStdVal = 0;			//AD���͊�l�i14bit(0x3ff=20mA)�j
	char	strStdVal[16];
	DWORD	ADInputVal[SP_XXXX_ADCH_MAX][AD_INPUT_DATA_NUM];
	DWORD	ADDatAve[SP_XXXX_ADCH_MAX][AD_INPUT_RANGE_CHK_NUM];	//[ch1�`4][4,12,20mA,OVR(L),OVR(H)]
	DWORD	ADDatMin[SP_XXXX_ADCH_MAX][AD_INPUT_RANGE_CHK_NUM];	//[ch1�`4][4,12,20mA,OVR(L),OVR(H)]
	DWORD	ADDatMax[SP_XXXX_ADCH_MAX][AD_INPUT_RANGE_CHK_NUM];	//[ch1�`4][4,12,20mA,OVR(L),OVR(H)]
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
		"������ %c:%s A/D���͐��x�m�F %s %s START ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);

	//�����@�����ڑ����A�i���O�d�����͌������[�h�ɐݒ肷��
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
	case SP_BID_UPPER_IF_BOARD:	//ch0�`ch2:4-20mA, ch3�`ch5:5V
		nAdChMax = 6;
		break;
	case SP_BID_FRONT_IF_BOARD:	//ch0�`ch3:4-20mA
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

	for( nChNum=nAdChMin; nChNum<nAdChMax; nChNum++ ){		//AD ch�����J��Ԃ�

		printf("\n-----------------------------------------\n");
		switch(m_nSPxxxxBoardID){
		case SP_BID_UPPER_IF_BOARD:
			nAdAdrOffs = 7;
			if( nChNum <= 2 ){		//ch0�`ch2:4-20mA
				nAdMode = AD_INPUT_RANGE_4_20mA;
				printf("[A/D ch%d]4-20mA���͂̐��x�m�F���J�n���܂����B", nChNum);
			}
			else{					//ch3�`ch5:5V
				nAdMode = AD_INPUT_RANGE_0_5V;
				printf("[A/D ch%d]0-5V���͂̐��x�m�F���J�n���܂����B", nChNum);
			}
			break;
		case SP_BID_FRONT_IF_BOARD:	//ch0�`ch3:4-20mA
			nAdAdrOffs = 7;
			nAdMode = AD_INPUT_RANGE_4_20mA;
			printf("[A/D ch%d]4-20mA���͂̐��x�m�F���J�n���܂����B", nChNum);
			break;
		case SP_BID_MOTHER_BOARD:	//ch0:10V
			if( m_nIniSpMotherBoardType == 0 ){
				nAdAdrOffs = 10;
			}
			else{
				nAdAdrOffs = 9;
			}
			nAdMode = AD_INPUT_RANGE_0_10V;
			printf("[A/D ch%d]0-10V���͂̐��x�m�F���J�n���܂����B", nChNum);
			break;
		}
		//CONTEC AIO���j�b�g����̏o�͊�l���Z�b�g����
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

		//ii=�ŏ��l(4mA, 0V),���Ԓl(12mA, 2.5V, 5V),�ő�l(20mA, 5V, 10V),
		//OVR_L(3.952mA,-0.015V,-0.03V),OVR_H(20.048mA, 5.015V, 10.03V)�̓��͐��x�m�F�����{
		for( ii=0; ii<AD_INPUT_RANGE_CHK_NUM; ii++ ){	//5��(MIN,MID,MAX,OVRL,OVRH)���[�v
			while(ForeverLoop){
				bKensaNg = TRUE;
				//���x�m�F�p��AD(14bit)���͊�l��ݒ肷��
				switch(ii){
				case 0:
					nIinStdVal = 0x0000;	//�ŏ��l
					break;
				case 1:
					nIinStdVal = 0x2000;	//���Ԓl
					break;
				case 2:
					nIinStdVal = 0x3FFF;	//�ő�l
					break;
				case 3:
					nIinStdVal = 0x0000;	//OVR(L)
					break;
				case 4:
					nIinStdVal = 0x3FFF;	//OVR(H)
					break;
				}
				
				printf("\n--------------------------------------------------\n");
				//printf("�V�O�i���\�[�X�E�}���`���[�^�������i��ch%d�ɐڑ����A\n", nChNum);

				switch(nAdMode){
				case AD_INPUT_RANGE_4_20mA:
					sprintf(strStdVal, "%4.2fmA", m_fAdInputStdVal[nAdMode][ii]);
					//printf("�V�O�i���\�[�X����%s���o�͂��Ă��������B\n", strStdVal);
					break;
				case AD_INPUT_RANGE_0_5V:
					sprintf(strStdVal, "%4.2fV", m_fAdInputStdVal[nAdMode][ii]);
					//printf("�V�O�i���\�[�X����%s���o�͂��Ă��������B\n", strStdVal);
					break;
				case AD_INPUT_RANGE_0_10V:
					sprintf(strStdVal, "%4.2fV", m_fAdInputStdVal[nAdMode][ii]);
					//printf("�V�O�i���\�[�X����%s���o�͂��Ă��������B\n", strStdVal);
					break;
				}

				//printf("----------------------------------------------------\n");
				//printf("�������ł����牽���L�[�������Ă��������B\n\n");
				//nInputVal = KeyInputWait();
				//if ( nInputVal == 0x1B ){
				//	break;
				//}

				//CONTEC AIO-160802AY-USB��AO00���璲���p�̓d�����o�͂���
				AioUnitWriteAOVal(0, (long)nAioUnitOutVal[ii]);
				printf("AD ch%d�̐��x�m�F���J�n���܂����B\n", nChNum);

				//AD���̓T���v�����O�J�n
				for( jj=0; jj<AD_INPUT_DATA_NUM; jj++ ){
					////RS-232C Mode 0�ŕ␳���A/D���͒l���擾����
					//ADInputVal[nChNum][jj] = (DWORD)RS_SpInputAdCorrectVal(nChNum);
					wTermAdr = (unsigned short)(nChNum+nAdAdrOffs);
					DATAin(BoardId, wTermAdr, (LPINT*)&dwInData);
					ADInputVal[nChNum][jj] = (DWORD)(dwInData & 0x3fff);

					//�擾�l���i���s�Ȃ��Łj��ʂɕ\������
					if( ii <= 2 ){			//4,12,20mA ���x�m�F
						printf("\r[%04d/%04d] AD ch%d[%s]=0x%04lX\r", 
								jj+1, AD_INPUT_DATA_NUM, nChNum, strStdVal, ADInputVal[nChNum][jj]);
					}
					else if( ii == 3 ){		//OVR(L)�m�F
						printf("\r[%04d/%04d] AD ch%d[OVR(L)]=0x%04lX\r", 
								jj+1, AD_INPUT_DATA_NUM, nChNum, ADInputVal[nChNum][jj]);
					}
					else{					//OVR(H)�m�F
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
					//AD���̓f�[�^�̍ő�E�ŏ��E���ς��擾����
					GetAdMinMaxAve(&ADInputVal[nChNum][0], &ADDatMin[nChNum][ii], &ADDatMax[nChNum][ii], &ADDatAve[nChNum][ii]);
					fwkmin[nChNum][ii] = (double)(fabs((double)(nIinStdVal - (int)ADDatMin[nChNum][ii])));
					fwkmin[nChNum][ii] = (double)(fwkmin[nChNum][ii] / (double)0x3fff) * 100.0;
					fwkave[nChNum][ii] = (double)(fabs((double)(nIinStdVal - (int)ADDatAve[nChNum][ii])));
					fwkave[nChNum][ii] = (double)(fwkave[nChNum][ii] / (double)0x3fff) * 100.0;
					fwkmax[nChNum][ii] = (double)(fabs((double)(nIinStdVal - (int)ADDatMax[nChNum][ii])));
					fwkmax[nChNum][ii] = (double)(fwkmax[nChNum][ii] / (double)0x3fff) * 100.0;

					//
					//���x�m�F�̌��ʂ�\������
					//
					//�ŏ��E���ԁE�ő�l�̓��͊m�F���͓��͐��x���}0.3%�ȓ��ɓ��邱�Ƃ��m�F����
					//(��)CONTEC AIO���j�b�g��10V�ȏ�̏o�͂͂ł��Ȃ����߁AOVR_H������0x3fff�\��t���`�F�b�N�ł͖����͈̓`�F�b�N���s���悤�ɂ���
					if( (ii <= 2) || ((nAdMode==AD_INPUT_RANGE_0_10V) && (ii == 4))){
						printf("\n<ch%d %s ADin>\nMIN=0x%04X(���x %f%% [%s])\nAVE=0x%04X(���x %f%% [%s])\nMAX=0x%04X(���x %f%% [%s])\n", 
								nChNum, strStdVal,
								ADDatMin[nChNum][ii], fwkmin[nChNum][ii], ((fwkmin[nChNum][ii] > AD_OK_RATE) ? "NG" : "OK"),
								ADDatAve[nChNum][ii], fwkave[nChNum][ii], ((fwkave[nChNum][ii] > AD_OK_RATE) ? "NG" : "OK"),
								ADDatMax[nChNum][ii], fwkmax[nChNum][ii], ((fwkmax[nChNum][ii] > AD_OK_RATE) ? "NG" : "OK") );

						if( fwkmin[nChNum][ii] > AD_OK_RATE || fwkave[nChNum][ii] > AD_OK_RATE || fwkmax[nChNum][ii] > AD_OK_RATE ){
							printf("ch%d��A/D���͐��x��%4.2f%%�𒴂��Ă��܂��B\a\n", nChNum, AD_OK_RATE);
							bKensaNg = FALSE;
						}

						LogPrintfCsv(m_strLogFileName, "----<(A/D ch%d) %s(0x%03X) ���͐��x�m�F>---\n", nChNum, strStdVal, nIinStdVal);
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
					//Over Range���͊m�F���́A���͒l������(0x0000)�A���(0x3fff)�ɓ\��t�����Ƃ��m�F����
					else{
						printf("\n<ch%d %s ADin>\nMIN=0x%04X[%s]\nAVE=0x%04X[%s]\nMAX=0x%04X[%s]\n", 
								nChNum, strStdVal,
								ADDatMin[nChNum][ii], ((ADDatMin[nChNum][ii] != (DWORD)nIinStdVal) ? "NG" : "OK"),
								ADDatAve[nChNum][ii], ((ADDatAve[nChNum][ii] != (DWORD)nIinStdVal) ? "NG" : "OK"),
								ADDatMax[nChNum][ii], ((ADDatMax[nChNum][ii] != (DWORD)nIinStdVal) ? "NG" : "OK") );
						if( (ADDatMin[nChNum][ii] != (DWORD)nIinStdVal) || (ADDatAve[nChNum][ii] != (DWORD)nIinStdVal) || (ADDatMax[nChNum][ii] != (DWORD)nIinStdVal) ){
							if( ii == 3 ){
								printf("ch%d��Over Range(L)�l��0x%X�ł͂���܂���B\a\n", nChNum, nIinStdVal);
							}
							else{
								printf("ch%d��Over Range(H)�l��0x%X�ł͂���܂���B\a\n", nChNum, nIinStdVal);
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
						printf("�Č������܂����H(y or n) -->\a\n");
						nInputVal = KeyInputWaitYorN();
						if( nInputVal == 'y' ){
							continue;			//�Č������{
						}
						else{
							bRetCont = FALSE;	//�Č��������{
						}
					}
				}

				break;
			}	//while end (����NG���̃��g���C�p)
		}	//for end (MIN,MID,MAX,OVR(L),OVR(H)��5�p�^�[���Ńe�X�g)

		if( nExitFlg == 1 ) break;
	}	//for end (AD ch�����J��Ԃ���)
	
	/////////////////////////////////////////////////////////////////////
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"A/D���͐��x�m�F�F%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	/////////////////////////////////////////////////////////////////////
	//A/D���͐��x�m�F�I������
	//RS_SpNormalMode();
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:%s A/D���͐��x�m�F %s %s FINISH ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

///////////////////////////////////////////////////////
//A/D���͒�������(SP_XXXX_BOARD�p)
// nInsID  : ����ID�ԍ�
// nSch    : A/D�ϊ��J�nch(-1�w�莞��Default��0�Ƃ���)
// nEch    : A/D�ϊ��I��ch(-1�w�莞��SP����Ɋ��蓖�Ă���ő�ch�ԍ��Ƃ���)
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
	DWORD	ADDatAve[SP_XXXX_ADCH_MAX][2];		//[ch0�`n][4,20mA]
	DWORD	ADDatMin[SP_XXXX_ADCH_MAX][2];		//[ch0�`n][4,20mA]
	DWORD	ADDatMax[SP_XXXX_ADCH_MAX][2];		//[ch0�`n][4,20mA]
	DWORD	dwAdOffset[SP_XXXX_ADCH_MAX];
	DWORD	dwAdGain[SP_XXXX_ADCH_MAX];
	DWORD	dwAdOffsetTemp[SP_XXXX_ADCH_MAX];
	DWORD	dwAdGainTemp[SP_XXXX_ADCH_MAX];
	BOOL	ADDataSet = FALSE;	//4mA�A20mA�Ƃ��T���v�����O�������TRUE���Z�b�g����
	int		nAOvalWork=m_nIniAO_0_0V;

	char	strDate[32];
	char	strTime[32];
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	lstrcpy(m_strLogFileName, "SPxxxx_AdChk.log");

	LogPrintfCsv(m_strLogFileName, 
		"������ %c:%s A/D���͒��� %s %s START ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);

	//�����@�����ڑ����A�i���O�d�����͌������[�h�ɐݒ肷��
	SnMultiMeterMeasureSel(0, ANA_INOUT_SEL_IN, ANA_VI_SEL_VOLT, ANA_LOAD_SEL_OPEN);
	
	memset( ADInputVal, 0, sizeof(ADInputVal) );

	for( ii=0; ii<SP_XXXX_ADCH_MAX; ii++ ){
		dwAdOffset[ii] = 0xffff;
		dwAdGain[ii] = 0xffff;
		dwAdOffsetTemp[ii] = 0xffff;
		dwAdGainTemp[ii] = 0xffff;
	}

	switch(m_nSPxxxxBoardID){
	case SP_BID_UPPER_IF_BOARD:	//ch0�`ch2:4-20mA, ch3�`ch5:5V
		nAdChMax = 6;
		break;
	case SP_BID_FRONT_IF_BOARD:	//ch0�`ch3:4-20mA
		nAdChMax = 4;
		break;
	case SP_BID_MOTHER_BOARD:	//ch0:10V
		nAdChMax = 1;
		break;
	default:
		printf("\nBOARD����ID���s���ł�\a\n");
		return FALSE;
	}
	if( nSch >= 0 ){
		nAdChMin = nSch;
	}
	if( nEch >= 0 ){
		nAdChMax = nEch + 1;
	}

	ADDataSet = FALSE;	//1ch�ł�4mA�A20mA�Ƃ��T���v�����O������������TRUE���Z�b�g����
	for( nChNum=nAdChMin; nChNum<nAdChMax; nChNum++ ){

		printf("\n-----------------------------------------\n");
		switch(m_nSPxxxxBoardID){
		case SP_BID_UPPER_IF_BOARD:
			if( nChNum <= 2 ){		//ch0�`ch2:4-20mA
				nAdMode = AD_INPUT_RANGE_4_20mA;
				printf("[A/D ch%d]4-20mA���͂̒������J�n���܂����B", nChNum);
			}
			else{					//ch3�`ch5:5V
				nAdMode = AD_INPUT_RANGE_0_5V;
				printf("[A/D ch%d]0-5V���͂̒������J�n���܂����B", nChNum);
			}
			break;
		case SP_BID_FRONT_IF_BOARD:	//ch0�`ch3:4-20mA
			nAdMode = AD_INPUT_RANGE_4_20mA;
			printf("[A/D ch%d]4-20mA���͂̒������J�n���܂����B", nChNum);
			break;
		case SP_BID_MOTHER_BOARD:	//ch0:10V
			nAdMode = AD_INPUT_RANGE_0_10V;
			printf("[A/D ch%d]0-10V���͂̒������J�n���܂����B", nChNum);
			break;
		}
		printf("\n-----------------------------------------\n");

		//A/D�������J�n����
		for( ii=0; ii<2; ii++ ){	//ii: 0=4mA(or 0V), 1=20mA(or 5V,10V)
			while(ForeverLoop){
				///////////////////////////////////////////////////////
				//�����i�̃A�i���O���͐��f�[�^������T���v�����O����
				ADDatAve[nChNum][ii] = 0;
				ADDatMax[nChNum][ii] = 0;
				ADDatMin[nChNum][ii] = 0xffff;

				printf("\n--------------------------------------------------\n");
				//printf("�V�O�i���\�[�X�E�}���`���[�^�������i��ch%d�ɐڑ����A\n", nChNum);

				switch(nAdMode){
				case AD_INPUT_RANGE_4_20mA:
					nAdVal = ((ii==0) ? 4 : 20);
					nAOvalWork = ((ii==0) ? m_nIniAO_4mA : m_nIniAO_20mA);
					nAdRawMinLimitVal = ((ii==0) ? INVALID_ADRAW_MINVAL_4mA : INVALID_ADRAW_MINVAL_20mA);
					nAdRawMaxLimitVal = ((ii==0) ? INVALID_ADRAW_MAXVAL_4mA : INVALID_ADRAW_MAXVAL_20mA);
					//printf("�V�O�i���\�[�X����%dmA���o�͂��Ă��������B\n", nAdVal);
					break;
				case AD_INPUT_RANGE_0_5V:
					nAdVal = ((ii==0) ? 0 : 5);
					nAOvalWork = ((ii==0) ? m_nIniAO_0_0V : m_nIniAO_5_0V);
					nAdRawMinLimitVal = ((ii==0) ? INVALID_ADRAW_MINVAL_F0V : INVALID_ADRAW_MINVAL_F5V);
					nAdRawMaxLimitVal = ((ii==0) ? INVALID_ADRAW_MAXVAL_F0V : INVALID_ADRAW_MAXVAL_F5V);
					//printf("�V�O�i���\�[�X����%dV���o�͂��Ă��������B\n", nAdVal);
					break;
				case AD_INPUT_RANGE_0_10V:
					nAdVal = ((ii==0) ? 0 : 10);
					nAOvalWork = ((ii==0) ? m_nIniAO_0_0V : m_nIniAO_10_0V);
					nAdRawMinLimitVal = ((ii==0) ? INVALID_ADRAW_MINVAL_T0V : INVALID_ADRAW_MINVAL_T10V);
					nAdRawMaxLimitVal = ((ii==0) ? INVALID_ADRAW_MAXVAL_T0V : INVALID_ADRAW_MAXVAL_T10V);
					//printf("�V�O�i���\�[�X����%dV���o�͂��Ă��������B\n", nAdVal);
					break;
				}

				//CONTEC AIO-160802AY-USB��AO00���璲���p�̓d�����o�͂���
				AioUnitWriteAOVal(0, (long)nAOvalWork);
				printf("AD ch%d�̐��f�[�^���͂��J�n���܂����B\n", nChNum);

				for( jj=0; jj<AD_INPUT_DATA_NUM; jj++ ){
					nAdRawVal = RS_SpInputAdRawVal(nChNum);

#ifdef	_RS_DEBUG_MODE
nAdRawVal = nAdRawMinLimitVal+0x100;
#endif
					if( nAdRawVal < nAdRawMinLimitVal ){
						printf("<<WARNING>> ch%d AD RAW Value 0x%04X (�l���Ⴗ����H)\n", nChNum, nAdRawVal);
						bWarnFlg = TRUE;
					}
					else if( nAdRawVal > nAdRawMaxLimitVal ){
						printf("<<WARNING>> ch%d AD RAW Value 0x%04X (�l����������H)\n", nChNum, nAdRawVal);
						if( nAdRawVal == 0xffff ){
							printf("AD���l���͂𒆎~���܂��B��낵���ł����H(y or n) -->");
							nInputVal = KeyInputWaitYorN();
							if( nInputVal == 'y' ){
								bRetCont = FALSE;
								nExitFlg = 1;
								break;
							}
						}
						bWarnFlg = TRUE;
					}
					else{	//���s�����ɓ��͂����l�����̂܂ܕ\������
						printf("\r[%04d/%04d] ch%d AD RAW Value=0x%04X\r",
								jj+1, AD_INPUT_DATA_NUM, nChNum, nAdRawVal);
					}
					ADInputVal[nChNum][jj] = (DWORD)nAdRawVal;
				}	//for jj�̏I���i�T���v�����O�����j
				printf("\n");

				if( bWarnFlg == TRUE ){	//���͒l�ُ�H�i�x������j�̂Ƃ�
					bWarnFlg = FALSE;
					switch(nAdMode){
					case AD_INPUT_RANGE_4_20mA:
						printf("[(ch%d)%dmA] ������xA/D���l���͂����{���܂����H(y or n) -->", nChNum, nAdVal);
						break;
					case AD_INPUT_RANGE_0_5V:
					case AD_INPUT_RANGE_0_10V:
						printf("[(ch%d)%dV] ������xA/D���l���͂����{���܂����H(y or n) -->", nChNum, nAdVal);
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
			}//Warning�������̌J��Ԃ��pwhile��end

			//AD���͐��f�[�^�̍ő�E�ŏ��E���ς��擾����
			if( (nExitFlg == 0) && (bRetCont == TRUE) ){
				GetAdMinMaxAve(&ADInputVal[nChNum][0], &ADDatMin[nChNum][ii], &ADDatMax[nChNum][ii], &ADDatAve[nChNum][ii]);
				if( ii == 1 ){	//�ő僌���W���̓��͎�
					//Offset/Gain�v�Z�l��\������
					GetAdOffsetGain(nAdMode, ADDatAve[nChNum][0], ADDatAve[nChNum][1], &dwAdOffset[nChNum], &dwAdGain[nChNum]);

					LogPrintfCsv(m_strLogFileName, "(ch%d RawVAL) Lo[0x%lX] Hi[0x%lX] Offset[0x%lX] Gain[0x%lX]\n",
								nChNum, ADDatAve[nChNum][0], ADDatAve[nChNum][1], dwAdOffset[nChNum], dwAdGain[nChNum] );
					printf("\n");
					ADDataSet = TRUE;		//�Sch 4,20mA�Ƃ����f�[�^�̃T���v�����O����
				}
			}
			if( nExitFlg == 1 ) break;

		}	//��ii=���͍ŏ������W���ƍő僌���W���̂Q���荞��
		
		if( nExitFlg == 1 ) break;
	}	//nChNum=AD ch�������[�v end

	/////////////////////////////////////////////////////////////////////
	//�T���v�����O�l�i���ϒl�̂ݎg�p�j�����ɂ��ĕ␳�l���Z�o����
	if( (nExitFlg == 0) && (ADDataSet == TRUE) && (bRetCont == TRUE) ){
		printf("\n-----------------------------------------\n");
		printf("�A�i���O���͕␳�l�X�V��...\n");

		if( RS_SpSetAdOffsetGain(dwAdOffset, dwAdGain, NULL, NULL) == TRUE ){
			//CSV�R�����g�s
			LogPrintfCsv(m_strLogFileName, "\n[AD_ch], [RawVAL(4mA)], [RawVAL(20mA)], [Offset], [Gain]\n");
			for( nChNum=nAdChMin; nChNum<nAdChMax; nChNum++ ){
				if( (dwAdOffset[nChNum] == 0xffff) || (dwAdGain[nChNum] == 0xffff) ){
					//�����ɂ͓���Ȃ��͂�
					printf("(ch%d) AD���͕␳ <�ύX�Ȃ�>�B\n", nChNum);
				}
				else{
					LogPrintfCsv(m_strLogFileName, "%d, %04lX, %04lX, %03lX, %03lX\n", 
								nChNum, ADDatAve[nChNum][0], ADDatAve[nChNum][1], dwAdOffset[nChNum], dwAdGain[nChNum]);
				}
			}
		}
		else{
			LogPrintfCsv(m_strLogFileName, "�A�i���O���͕␳�l�X�V�s��\n");
			bRetCont = FALSE;
		}
	}
	
	/////////////////////////////////////////////////////////////////////
	//A/D���͕␳�l��EEPROM�����݊m�F
	if( (nExitFlg == 0) && (ADDataSet == TRUE) && (bRetCont == TRUE) ){
		LogPrintfCsv(m_strLogFileName, "\n<<<EEPROM SAVE>>>\n");
		while(ForeverLoop){
			printf("\n-----------------------------------------\n");
			printf("EEPROM�ۑ����s��...\n");
			bRetCont = TRUE;
			if( RS_SpUpdateEeprom(nAdChMax) == TRUE ){		//EEPROM�ۑ����s
				printf("EEPROM�ۑ����������܂����B\n\n");
				printf("\n-----------------------------------------\n");
				printf("EEPROM�ɕۑ����ꂽ�f�[�^���m�F���܂��B�����i�̓d�����ē������Ă��������B\n");
				printf("�������ł����牽���L�[�������Ă��������B");
				KeyInputWait();
				printf("\n");
				if( RS_SpSetAdOffsetGain(dwAdOffsetTemp, dwAdGainTemp, dwAdOffsetTemp, dwAdGainTemp) == TRUE ){	//�␳�l�̓Ǐo���̂ݎ��s
					for( nChNum=nAdChMin; nChNum<nAdChMax; nChNum++ ){
						if( (dwAdOffset[nChNum] == 0xffff || dwAdGain[nChNum] == 0xffff) ||
							(dwAdOffsetTemp[nChNum] == dwAdOffset[nChNum] && dwAdGainTemp[nChNum] == dwAdGain[nChNum]) )
						{
							LogPrintfCsv(m_strLogFileName, " [OK]:(ch%d) Offset W[0x%04X] R[0x%04X], GAIN W[0x%04X] R[0x%04X]\n",
										nChNum, dwAdOffset[nChNum], dwAdOffsetTemp[nChNum],
										dwAdGain[nChNum], dwAdGainTemp[nChNum]);
						}
						else{
							//AD���͕␳�l����v���܂���B
							bRetCont = FALSE;
							LogPrintfCsv(m_strLogFileName, " [NG]:(ch%d) Offset W[0x%04X] R[0x%04X], GAIN W[0x%04X] R[0x%04X]\n",
										nChNum, dwAdOffset[nChNum], dwAdOffsetTemp[nChNum],
										dwAdGain[nChNum], dwAdGainTemp[nChNum]);
						}
					}
				}
				else{
					printf("AD���͕␳�l���擾�ł��܂���B\n");
					bRetCont = FALSE;
				}
				if( bRetCont != TRUE ){
					printf("AD���͕␳�l��EEPROM�ւ̏������݂Ɏ��s���܂����B�ēx�A���s���܂����H(y or n) -->");
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						continue;
					}
				}
			}
			else{
				printf("EEPROM�ۑ��Ɏ��s���܂����B�ēx�A���s���܂����H(y or n) -->");
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
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"A/D���͒����I�������F%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	/////////////////////////////////////////////////////////////////////
	//A/D���͒����I������
	RS_SpNormalMode();
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:%s A/D���͒��� %s %s FINISH ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

///////////////////////////////////////////////////////
//LED�ڎ��E����d���m�F
// nInsID  : ����ID�ԍ�
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
		"������ %c:%s LED�ڎ��m�F�^����d���m�F %s %s START ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);

	///////////////////////////////////////
	//LED�̖ڎ����������{����
	while( bRetCont == TRUE ){	//�Č����pwhile
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("  ��LED�ڎ��m�F��\n");
		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
			printf("   LD1�`LD5�̗�LED���_�����Ă��邱�Ƃ��m�F���Ă�������\n");
		}
		else if( m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ){
			printf("   LD1�`LD7�̗�LED���_�����Ă��邱�Ƃ��m�F���Ă�������\n");
		}
		else if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			if( m_nIniSpMotherBoardType == 0 ){		//SP_MOTHER_BOARD
				printf("   LD1�`LD9�̗�LED���_�����Ă��邱�Ƃ��m�F���Ă�������\n");
			}
			else{
				printf("   LD1�`LD4�ALD6�`LD9�̗�LED���_�����Ă��邱�Ƃ��m�F���Ă�������\n");
			}
		}
		printf("------------------------------------------------------\n");
		printf("\n");
		printf(" LED�̓`���c�L�E�P�x�s���Ȃ�����ɓ_�����Ă��܂����H(y or n) -->");
		nInputVal = KeyInputWaitYorN();
		if( nInputVal == 0x1B ){
			//�������f
			LogPrintfCsv(m_strLogFileName, "[Cancel] LED�_���ڎ��m�F\n");
			bRetCont = FALSE;
			break;
		}
		else if( nInputVal == 'y' ){
			//����OK
			LogPrintfCsv(m_strLogFileName, "[OK] LED�_���ڎ��m�F\n");
		}
		else{
			//����NG
			LogPrintfCsv(m_strLogFileName, "[NG] LED�_���ڎ��m�F\n");
			bRetCont = FALSE;
		}

		if( bRetCont != TRUE ){
			printf("�Č������܂����H(y or n) -->");
			nInputVal = KeyInputWaitYorN();
			if( nInputVal == 'y' ){
				bRetCont = TRUE;
				LogPrintfCsv(m_strLogFileName, "\n----�Č���----\n");
				continue;			//�Č������{
			}
		}
		break;
	}	//while (�Č����p)

	///////////////////////////////////////
	//����d���̊m�F�����{����
	while( bRetCont == TRUE ){	//�Č����pwhile
		printf("\n");
		printf("------------------------------------------------------\n");
		printf("  ��DC24V����d���m�F��\n");
		if( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
				printf("   �����i�̎�d��(DC24V)�̏���d����250mA�ȉ��ł��邱�Ƃ��m�F���Ă�������\n");
		}
		else if( m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ){
				printf("   �����i�̎�d��(DC24V)�̏���d����250mA�ȉ��ł��邱�Ƃ��m�F���Ă�������\n");
		}
		else if( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
			if( m_nIniSpMotherBoardType == 0 ){		//SP_MOTHER_BOARD
				printf("   �����i�̎�d��(DC24V)�̏���d����250mA�ȉ��ł��邱�Ƃ��m�F���Ă�������\n");
			}
			else{	//SPIN_MOTHER_BOARD
				printf("   �����i�̎�d��(DC24V)�̏���d����280mA�ȉ��ł��邱�Ƃ��m�F���Ă�������\n");
			}
		}
		printf("------------------------------------------------------\n");
		printf("\n");
		printf(" ����d���͐���ł����H(y or n) -->");
		nInputVal = KeyInputWaitYorN();
		if( nInputVal == 0x1B ){
			//�������f
			LogPrintfCsv(m_strLogFileName, "[Cancel] ����d���m�F\n");
			bRetCont = FALSE;
			break;
		}
		else if( nInputVal == 'y' ){
			//����OK
			LogPrintfCsv(m_strLogFileName, "[OK] ����d���m�F\n");
		}
		else{
			//����NG
			LogPrintfCsv(m_strLogFileName, "[NG] ����d���m�F\n");
			bRetCont = FALSE;
		}

		if( bRetCont != TRUE ){
			printf("�Č������܂����H(y or n) -->");
			nInputVal = KeyInputWaitYorN();
			if( nInputVal == 'y' ){
				bRetCont = TRUE;
				LogPrintfCsv(m_strLogFileName, "\n----�Č���----\n");
				continue;			//�Č������{
			}
		}
		break;
	}	//while (�Č����p)

	/////////////////////////////////////////////////////////////////////
	// �������ʏo��
	/////////////////////////////////////////////////////////////////////
	printf("��������������������������������������������\n");
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	LogPrintfCsv(m_strLogFileName, 
		"LED�ڎ��m�F�E����d���m�F�F%s\n", ((bRetCont==TRUE) ? ("OK") : ("NG")));
	LogPrintfCsv(m_strLogFileName,
		"--------------------------------------------\n");
	printf("��������������������������������������������\n");

	/////////////////////////////////////////////////////////////////////
	//�I������
	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv(m_strLogFileName, 
		"������ %c:%s LED�ڎ��m�F�^����d���m�F %s %s FINISH ������\n", nInsID, m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

//�w��A�h���X�͈͂̒ʐM��Ԃ��m�F����
//bCommChk: TRUE=�ʐM���ł��邱�Ƃ��m�F�AFALSE=��ʐM��Ԃł��邱�Ƃ��m�F
BOOL SpComChk(int nStartAdr, int nEndAdr, BOOL bCommChk)
{
	BOOL	berr = TRUE;
	DWORD	err;
	DWORD	termerr;            // �^�[�~�i���ʐM���
	WORD	unit;

	for( unit=(WORD)nStartAdr; unit<=(WORD)nEndAdr; unit++ ){
		if( unit > 0 ){
			err = TERMchk(BoardId, unit, (LPINT*)&termerr);
			if(err != SNMA_NO_ERROR){
				printf("\nFailed to Get Terminal Communication Status!!\n");
				printf("�����L�[�������Ă��������B\a\n");
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
//D/A�o�͒�������(SP_XXXX_BOARD�p)
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
		"������ [%s] D/A�o�͒��� %s %s START ������\n", m_cSPxxxxBoardName, strDate, strTime);

	//�}���`���[�^�Ōv�����邽�߂̏���
	retStart_HP34401A = Start_HP34401A(
		m_cComPort,
		&forMultiMaterHandles	//�ʐM���\�[�X�̃n���h����Ԃ��A�h���X
	);
	if( ! retStart_HP34401A ){
		LogPrintf("MultiMeter 34401A Initialize Error!!\n");
		return FALSE;
	}

	//�}���`���[�^�̃����W��d�����͂ɐݒ肷��
	retChangeVA_DC_HP34401A = ChangeVoltageDC_HP34401A(
		forMultiMaterHandles	//�ʐM���\�[�X�n���h��
	);
	if( ! retChangeVA_DC_HP34401A ){
		LogPrintf("MultiMeter 34401A Voltage Setting Error!!\n");
		End_HP34401A(forMultiMaterHandles);	//�ʐM���\�[�X�̃n���h�������
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

	if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) == TRUE ){	//�␳�l�i���ݒl�j�̓Ǐo���̂ݎ��s
		memcpy(dwDaOffset_OldVal, dwDaOffset, sizeof(dwDaOffset_OldVal));
		memcpy(dwDaGain_OldVal, dwDaGain, sizeof(dwDaGain_OldVal));

		for( nChNum=0; nChNum<SP_XXXX_DACH_MAX; nChNum++ ){
			printf("\n---------------------------------------------------------\n");
			printf(" �}���`���[�^�̃v���[�u��DA ch%d�[�q�ɐڑ����Ă��������B\n", nChNum);
			printf("-----------------------------------------------------------\n");
			printf("�������ł����牽���L�[�������Ă��������B\n\n");
			nInputVal = KeyInputWait();
			if ( nInputVal == 0x1B ){
				//return FALSE;
				bRetCont = FALSE;
				break;
			}

			while(ForeverLoop){	//�������g���C�pwhile
				//[���[�h3]�A�i���O�o�͕␳���[�h�ɐ؂�ւ��āA�I�t�Z�b�g�X�V�\�ȏ�ԂƂ���
				printf("[D/A ch%d] Offset=0x%04X, Gain=0x%04X\n", nChNum, dwDaOffset[nChNum], dwDaGain[nChNum]);

				//printf("\n-----------------------------------------------------------------\n");
				//printf(" SN�f���\�t�g����MKY(Adr%d)��0V(0x0000)�o�͌�ɒ������J�n���Ă�������\n", nChNum+(SP_XXXX_SW2_START_ADR+2));
				//printf("------------------------------------------------------------------\n");

				wTermAdr = (unsigned short)(nChNum+(SP_XXXX_SW2_START_ADR+2));
				err = DATAout(BoardId, wTermAdr, (WORD)0x0000);
				if( err != SNMA_NO_ERROR ){
					printf("DATAout() err=%ld\n", err);
					printf("�����L�[�������Ă��������B\a\n");
					HitAnyKeyWait();
					return FALSE;
				}
				printf("\n-----------------------------------------------------------------\n");
				printf(" MKY(Adr%d)��0V(0x0000)�o�͂��J�n���܂���\n", wTermAdr);
				printf("------------------------------------------------------------------\n");

				//�o�̓f�[�^��D11 ��D0�̏��ɂP�r�b�g����ON���Ă����A�}���`���[�^����̓��͒l��
				//0V�����ɂȂ��OFF�ɖ߂��āA���̃r�b�g�Ɉڂ�
				if( bRetCont == TRUE ){
					printf("\nD/A(ch%d) �␳�l(Offset)�̒�����...\n\n", nChNum);
					dwDaOffset[nChNum] = 0x0000;
					dwDaGain[nChNum] = 0x0400;

					dwDaOffset_GoodVal = dwDaOffset[nChNum];
					dwDaGain_GoodVal = dwDaGain[nChNum];
					fVoltage_GoodVal = -999.9;

					for( ii=11; ii>=0; ii-- ){
						dwTemp = dwDaOffset[nChNum];
						dwDaOffset[nChNum] = dwTemp | (DWORD)(0x0001 << ii);
						//Offset�␳�l���X�V����
						if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//�ύX��̒l�ɍX�V����
							printf("[D/A ch%d] Offset�␳�l�̍X�V�Ɏ��s���܂����B\a\a\n", nChNum);
							bRetCont = FALSE;
							break;
						}

						//�}���`���[�^����d���l���擾����
						Sleep_Cnt(1000);
						if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
							printf("�d���l���擾�ł��܂���ł����B�����L�[�������Ă��������B\a\n");
							HitAnyKeyWait();
							bRetCont = FALSE;
							break;
						}
						printf("Offset=0x%04X, Gain=0x%04X [%fV]\n", dwDaOffset[nChNum], dwDaGain[nChNum], fVoltageVal);

						//�ڕW�l(0.0V�ɍł��߂�����OFFSET/GAIN��ϐ�GoodVal�ɋL�����Ă���)
						if( (double)fabs(fVoltageVal - 0.0) < (double)fabs(fVoltage_GoodVal - 0.0) ){
							dwDaOffset_GoodVal = dwDaOffset[nChNum];
							dwDaGain_GoodVal = dwDaGain[nChNum];
							fVoltage_GoodVal = fVoltageVal;
						}
						
						if( fVoltageVal > 1.0 ){
							printf("�����i��ch%d �����D/A�o�͒l���������܂��B\a\n", nChNum);
							printf("�o�͒l(0x0000), ������l(0.0 V), �}���`���[�^�l(%f V)\n", fVoltageVal);
							printf("hit any key.\a\n");
							HitAnyKeyWait();
							bRetCont = FALSE;
							break;
						}
						else if( fVoltageVal < 0.0 ){
							//���͒l��0V�����Ȃ�Ό��ɖ߂��i����bit��OFF����j
							dwDaOffset[nChNum] = dwTemp;
						}

						if( ii == 0 ){	//�ŏIbit�̂Ƃ�
							dwDaOffset[nChNum] = dwDaOffset_GoodVal;
							dwDaGain[nChNum] = dwDaGain_GoodVal;
							if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//�ύX��̒l�ɍX�V����
								printf("[D/A ch%d] Offset�␳�l�̍X�V�Ɏ��s���܂����B\a\a\n", nChNum);
								bRetCont = FALSE;
								break;
							}

							//�}���`���[�^����ŏI�̓d���l���擾����
							Sleep_Cnt(1000);
							if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
								printf("�d���l���擾�ł��܂���ł����B�����L�[�������Ă��������B\a\n");
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
				//printf(" SN�f���\�t�g����MKY(Adr%d)��5V(0x3FFF)�o�͌�ɒ������J�n���Ă�������\n", nChNum+(SP_XXXX_SW2_START_ADR+2));
				//printf("---------------------------------------------------------\n");
				err = DATAout(BoardId, wTermAdr, (WORD)0x3FFF);
				if( err != SNMA_NO_ERROR ){
					printf("DATAout() err=%ld\n", err);
					printf("�����L�[�������Ă��������B\a\n");
					HitAnyKeyWait();
					bRetCont = FALSE;
					break;
				}
				printf("\n-----------------------------------------------------------------\n");
				printf(" MKY(Adr%d)��5V(0x3FFF)�o�͂��J�n���܂���\n", wTermAdr);
				printf("------------------------------------------------------------------\n");

				//�o�̓f�[�^��D11 ��D0�̏��ɂP�r�b�g����ON���Ă����A�}���`���[�^����̓��͒l��
				//5V�𒴂�����OFF�ɖ߂��āA���̃r�b�g�Ɉڂ�
				if( bRetCont == TRUE ){
					printf("\nD/A(ch%d) �␳�l(Gain)�̒�����...\n\n", nChNum);
					dwDaGain[nChNum] = 0x0000;

					dwDaOffset_GoodVal = dwDaOffset[nChNum];
					dwDaGain_GoodVal = dwDaGain[nChNum];
					fVoltage_GoodVal = -999.9;

					for( ii=11; ii>=0; ii-- ){
						dwTemp = dwDaGain[nChNum];
						dwDaGain[nChNum] = dwTemp | (DWORD)(0x0001 << ii);
						//Gain�␳�l���X�V����
						if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//�ύX��̒l�ɍX�V����
							printf("[D/A ch%d] Offset�␳�l�̍X�V�Ɏ��s���܂����B\a\a\n", nChNum);
							bRetCont = FALSE;
							break;
						}

						//�}���`���[�^����d���l���擾����
						Sleep_Cnt(1000);
						if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
							printf("�d���l���擾�ł��܂���ł����B�����L�[�������Ă��������B\a\n");
							HitAnyKeyWait();
							bRetCont = FALSE;
							break;
						}

						printf("Offset=0x%04X, Gain=0x%04X [%fV]\n", dwDaOffset[nChNum], dwDaGain[nChNum], fVoltageVal);

						//�ڕW�l(5.0V�ɍł��߂�����OFFSET/GAIN��ϐ�GoodVal�ɋL�����Ă���)
						if( (double)fabs(fVoltageVal - 5.0) < (double)fabs(fVoltage_GoodVal - 5.0) ){
							dwDaOffset_GoodVal = dwDaOffset[nChNum];
							dwDaGain_GoodVal = dwDaGain[nChNum];
							fVoltage_GoodVal = fVoltageVal;
						}

						if( fVoltageVal < 4.5 || fVoltageVal > 5.5){
							if( fVoltageVal < 4.5 ){
								printf("�����i��ch%d �����D/A�o�͒l���Ⴗ���܂��B\a\n", nChNum);
							}
							else{
								printf("�����i��ch%d �����D/A�o�͒l���������܂��B\a\n", nChNum);
							}
							printf("�o�͒l(0x3FFF), ������l(5.0 V), �}���`���[�^�l(%f V)\n", fVoltageVal);
							printf("hit any key.\a\n");
							HitAnyKeyWait();
							bRetCont = FALSE;
							break;
						}
						else if( fVoltageVal > 5.0 ){
							//���͒l��5V�𒴂����猳�ɖ߂��i����bit��OFF����j
							dwDaGain[nChNum] = dwTemp;
						}
						else if( ii == 11 ){
							//----------------------------------------------------------------------
							//�� SP_UPER_IF_BOARD�ł͍��̂Ƃ���5V�ȉ��̊�������̂ł��̏����͕ۗ�
							//----------------------------------------------------------------------
							//�ŏ��̃r�b�g�i11�r�b�g�ځjON�ŁA���͒l��5V�𒴂��Ȃ��ꍇ�́A
							//�ő�l0x0fff��ݒ��A�l���ω������r�b�g�݂̂�L���ɂ���
							printf("[DebInfo]0xFFF Output Value = %fV (<5V)\n", fVoltageVal);
							//if( DaGainAdj_1(forMultiMaterHandles, nChNum, &wDoVal, &fCurVal) != TRUE ){
								printf("�����L�[�������Ă��������B\a\n");
								HitAnyKeyWait();
								bRetCont = FALSE;		//5V���o���Ȃ���͍��̂Ƃ���G���[�ɂ��Ă���
							//}
							break;
						}

						if( ii == 0 ){	//�ŏIbit�̂Ƃ�
							dwDaOffset[nChNum] = dwDaOffset_GoodVal;
							dwDaGain[nChNum] = dwDaGain_GoodVal;
							if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//�ύX��̒l�ɍX�V����
								printf("[D/A ch%d] Gain�␳�l�̍X�V�Ɏ��s���܂����B\a\a\n", nChNum);
								bRetCont = FALSE;
								break;
							}

							//�}���`���[�^����ŏI�̓d���l���擾����
							Sleep_Cnt(1000);
							if( GetVoltageVal(forMultiMaterHandles, &fVoltageVal) != TRUE ){
								printf("�d���l���擾�ł��܂���ł����B�����L�[�������Ă��������B\a\n");
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
					printf("[ch%d(0x%04X)] 5.0V�ɂȂ�悤��Gain�l�𒲐����Ă��������i�I��:q�j=>", nChNum, dwDaGain[nChNum]);
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
							if( RS_SpSetDaOffsetGain(dwDaOffset, dwDaGain, dwDaOffset, dwDaGain) != TRUE ){	//�ύX��̒l�ɍX�V����
								printf("[D/A ch%d] Gain�␳�l�̍X�V�Ɏ��s���܂����B\a\a\n", nChNum);
							}
						}
					}
				}
*/
				if( bRetCont == TRUE ){
					printf("\nD/A ch%d�̒������I�����܂����H (y or n) -->", nChNum);
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
			}	//�������g���C�pwhile�̏I���
			if( bRetCont != TRUE ) break;
		}	//D/A ch�����J��Ԃ�

		//���ʂ��o�͂���
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
			//printf("\nEEPROM��D/A�␳�l��ۑ����܂����H (y or n) -->");
			//nInputVal = KeyInputWait();
			//printf("\n");
			//if( nInputVal == 'y' || nInputVal == 'Y' || nInputVal == 0x0d ){
				printf("\n-----------------------------------------\n");
				printf("EEPROM�ۑ����s��...\n");
				bRetCont = TRUE;
				if( RS_SpUpdateEeprom(SP_XXXX_DACH_MAX) == TRUE ){		//EEPROM�ۑ����s
					printf("EEPROM�ۑ����������܂����B\n\n");
					printf("\n-----------------------------------------\n");
					printf("EEPROM�ɕۑ����ꂽ�f�[�^���m�F���܂��B�����i�̓d�����ē������Ă��������B\n");
					printf("�������ł����牽���L�[�������Ă��������B");
					KeyInputWait();
					printf("\n");

					if( RS_SpSetDaOffsetGain(dwDaOffsetTemp, dwDaGainTemp, dwDaOffsetTemp, dwDaGainTemp) == TRUE ){	//�␳�l�̓Ǐo���̂ݎ��s
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
								//DA�o�͕␳�l����v���܂���B
								bRetCont = FALSE;
								LogPrintfCsv("SPxxxx_DaLog.csv", " [NG]:(ch%d) Offset W[0x%04X] R[0x%04X], GAIN W[0x%04X] R[0x%04X]\n",
											nChNum, dwDaOffset[nChNum], dwDaOffsetTemp[nChNum],
											dwDaGain[nChNum], dwDaGainTemp[nChNum]);
							}
						}
					}
					else{
						printf("DA�o�͕␳�l���擾�ł��܂���B\a\a\n");
						bRetCont = FALSE;
					}
				}
			//}
		}
	}
	else{
		//LogPrintfCsv("SPxxxx_DaLog.csv", "\n");
		printf("[D/A] �o�͕␳�l�̎擾�Ɏ��s���܂����B\a\a\n");
		bRetCont = FALSE;
		//break;
	}

	/////////////////////////////////////////////////////////////////////
	//D/A�o�͒����I������
	RS_SpNormalMode();

	//�}���`���[�^�ʐM���\�[�X�̃n���h�������
	if( forMultiMaterHandles != NULL ){
		End_HP34401A(forMultiMaterHandles);
	}

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv("SPxxxx_DaLog.csv", 
		"������ [%s] D/A�o�͒��� %s %s FINISH ������\n", m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;
}

//////////////////////////////////////////////////////////////
//SAVENET�𗘗p�������o�̓e�X�g
//////////////////////////////////////////////////////////////

////////////////////////////////////////////
//SP_UPPER_IF_BOARD �����łł��Ȃ�DIO����
BOOL SPxxxxDioChkManualUpper1()
{
	return SPxxxxDioChkManualUpper2();
	//return TRUE;
}

////////////////////////////////////////////
//SP_UPPER_IF_BOARD2 �����łł��Ȃ�DIO����
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
	//IN3-8���̓`�F�b�N
	while( bRetCont == TRUE ){
		printf("\n---------------------------------------------------------\n");
		printf(" ��IN3-8���͌����J�n��\n" );
		printf(" CN34��IN3_8���̓{�^�������������Ă�������\n" );
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
								"IN3-8���̓G���[(0x%04X)\n", (WORD)dwInData[0]);
					break;
				}
				Sleep_Cnt(5);
			}while((DWORD)(GetTickCount()-dwStartTick) < 200);
		}

		if( bRetCont == TRUE ){
			printf("\n----------------------------------------------\n");
			printf(" IN3_8���̓{�^���𑀍삵��CN33,CN34�̗�LED�_���E������Ԃ�ڎ��m�F���Ă�������\n" );
			printf(" �{�^���������͓_�����A�{�^�������𗣂��Ə������܂����H(y or n)-->\a" );
			//printf(" IN3_8���̓{�^���̉�������CN33,CN34�̗�LED���_�����Ă��܂����H(y or n) -->\n" );
			nInputVal = KeyInputWaitYorN();
			if( nInputVal != 'y' ){
				printf("\n");
				bRetCont = FALSE;
				LogPrintfCsv("SPxxxx_DioLog.csv", "IN3-8(SW_ON)����ON/OFF�A���e�X�g�G���[(LED_ON,LIGHT_ON�o�̓G���[)\n");
			}
			else{
				printf("\n");
			}
		}
		
		//if( bRetCont == TRUE ){
		//	printf("\n----------------------------------------------\n");
		//	printf(" �{�^�������𗣂��ƁACN33,CN34�̗�LED�͏������܂������H(y or n) -->\n" );
		//	nInputVal = KeyInputWait();
		//	if( nInputVal != 'y' && nInputVal != 'Y' && nInputVal != 0x0d ){
		//		printf(" n\n\n");
		//		bRetCont = FALSE;
		//		LogPrintfCsv("SPxxxx_DioLog.csv", "IN3-8(SW_ON)����OFF�A���e�X�g�G���[(LED_ON,LIGHT_ON�o�̓G���[)\n");
		//	}
		//	else{
		//		printf(" y\n\n");
		//	}
		//}	
			
		if( bRetCont == TRUE ){
			printf("\n----------------------------------------------\n");
			LogPrintfCsv("SPxxxx_DioLog.csv", "IN3_8 ���͌���OK\n");
			printf("----------------------------------------------\n\n");
		}
		else{
			printf("\n----------------------------------------------\n");
			LogPrintfCsv("SPxxxx_DioLog.csv", "IN3_8 ���͌���NG\n");
			printf("----------------------------------------------\n\n");
			printf("�ēx�AIN3_8 ���̓e�X�g���s���܂����H(y or n) -->");
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
		//IN7-e,f IN8-e,f IN9-e,f(ALM_A, ALM_B)���̓`�F�b�N�AZEROSET�o�̓`�F�b�N
		//if(0){
		while( bRetCont == TRUE ){
			printf("\n---------------------------------------------------------\n");
			printf(" ��ALM_A, ALM_B, SIG ���͌����J�n��\n" );
			printf("-----------------------------------------------------------\n");
			
			for( ii=0; ii<2; ii++ ){	//DSW ON,OFF����ɂ�鏈���̐؂�ւ�
				if( bRetCont == TRUE ){
					printf("\n\n");
					printf("������������������������������������������������������������\n");
					printf("  [�e�X�g����]\a\n");
					printf("  ����DSW1�`DSW3��%s�ɂ�����A�w���ɏ]���đ��삵�Ă�������\n", ((ii==0) ? "ON": "OFF"));
					printf("������������������������������������������������������������\n\n");
					//nInputVal = KeyInputWait();
					//if ( nInputVal == 0x1B ){
					//	bRetCont = FALSE;
					//}

					for( jj=0; jj<3; jj++ ){	//SW�O���[�v���ɏ���
						if( bRetCont == TRUE ){
							TestOkCount = 0;
							while((TestOkCount<2) && (bRetCont == TRUE)){	//���������2��J��Ԃ�
								kk=0;
								while(kk<3){		//�{�^�����ɏ���
								//for( kk=0; kk<3; kk++ ){	//�{�^�����ɏ���
									//���҂�����͒ldwIN7Val�`dwIN9Val�ɃZ�b�g����
									switch(jj){
									case 0:	//1�Ԗڂ�SW�O���[�v
										if( ii == 0 ){	//DSW��ON�̂Ƃ�
											dwIN7Val = 0xC000;
											dwIN8Val = 0x0000;
											dwIN9Val = 0x0000;
										}
										else{			//DSW��OFF�̂Ƃ�
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
									case 1:	//2�Ԗڂ�SW�O���[�v
										if( ii == 0 ){	//DSW��ON�̂Ƃ�
											dwIN7Val = 0x0000;
											dwIN8Val = 0xC000;
											dwIN9Val = 0x0000;
										}
										else{			//DSW��OFF�̂Ƃ�
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
									case 2:	//3�Ԗڂ�SW�O���[�v
										if( ii == 0 ){	//DSW��ON�̂Ƃ�
											dwIN7Val = 0x0000;
											dwIN8Val = 0x0000;
											dwIN9Val = 0xC000;
										}
										else{			//DSW��OFF�̂Ƃ�
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
										//printf(" SW�����𗣂��Ă�������\n");
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
									printf(" [�e�X�g%d���] %d�Ԗڂ�%s�{�^�������������Ă�������\n", TestOkCount+1, jj+1, ((kk==0) ? "ALM_A": ((kk==1) ? "ALM_B" : "DIN")));
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
												"[SW%d] DSW_%s�� %s���̓G���[ IN7(0x%04X) IN8(0x%04X) IN9(0x%04X)\n",
												jj+1, ((ii==0) ? "ON" : "OFF"),
												((kk==0) ? "ALM_A": ((kk==1) ? "ALM_B" : "DIN")),
												(WORD)dwInData[0], (WORD)dwInData[1], (WORD)dwInData[2]);
											printf("������x�������܂����H(y or n) -->");
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
										printf(" SW ���������o���܂���\n");
									}
									if( bRetCont != TRUE ){
										break;
									}
								} //�{�^�����ɏ���

								if( bRetCont == TRUE ){
									TestOkCount++;
								}

							}	//�����e�X�g���Q��J��Ԃ�
						}
					}
				}
			}
			if( bRetCont == TRUE ){
				printf("\n----------------------------------------------\n");
				LogPrintfCsv("SPxxxx_DioLog.csv", "ALM_A, ALM_B, SIG ���͌��� OK\n");
				printf("----------------------------------------------\n");
			}
			else{
				printf("\n----------------------------------------------\n");
				LogPrintfCsv("SPxxxx_DioLog.csv", "ALM_A, ALM_B, SIG ���͌��� NG\n");
				printf("----------------------------------------------\n");
				printf("�ēx�AALM_A, ALM_B, SIG ���͌������s���܂����H(y or n) -->");
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
			printf(" ��ZEROSET �o�͌����J�n��\n" );
			printf("-----------------------------------------------------------\n");
			for( jj=0; jj<3; jj++ ){	//SW�O���[�v���ɏ���
				if( bRetCont == TRUE ){
					DATAout(BoardId, SP_XXXX_SW1_START_ADR+6, 0x0000);
					DATAout(BoardId, SP_XXXX_SW1_START_ADR+7, 0x0000);
					DATAout(BoardId, SP_XXXX_SW1_START_ADR+8, 0x0000);
					DATAout(BoardId, (WORD)(SP_XXXX_SW1_START_ADR+6+jj), 0x8000);
					printf("%d�Ԗڂ�ZEROSET(��LED)�̂ݓ_�����Ă��܂����H(y or n) -->", jj+1);
					nInputVal = KeyInputWaitYorN();
					if( nInputVal != 'y' ){
						bRetCont = FALSE;
						LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET (ADR%d) �o�͖ڎ����� NG\n", SP_XXXX_SW1_START_ADR+6+jj);
					}
				}
			}
			if( bRetCont == TRUE ){
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+6, 0x8000);
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+7, 0x8000);
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+8, 0x8000);
				printf("ZEROSET(��LED)�͑S3�Ƃ��_�����Ă��܂����H(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal != 'y' ){
					bRetCont = FALSE;
					LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET �o�͖ڎ�����(ALL ON) NG\n");
				}
			}
			if( bRetCont == TRUE ){
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+6, 0x0000);
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+7, 0x0000);
				DATAout(BoardId, SP_XXXX_SW1_START_ADR+8, 0x0000);
				printf("ZEROSET(��LED)�͑S3�Ƃ��������܂������H(y or n) -->");
				nInputVal = KeyInputWaitYorN();
				if( nInputVal != 'y' ){
					bRetCont = FALSE;
					LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET �o�͖ڎ�����(ALL OFF) NG\n");
				}
			}

			if( bRetCont == TRUE ){
				printf("\n----------------------------------------------\n");
				LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET �o�͖ڎ����� OK\n");
				printf("----------------------------------------------\n");
			}
			else{
				printf("\n----------------------------------------------\n");
				LogPrintfCsv("SPxxxx_DioLog.csv", "ZEROSET �o�͖ڎ����� NG\n");
				printf("----------------------------------------------\n");
				printf("�ēx�AZEROSET �o�͌������s���܂����H(y or n) -->");
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
		LogPrintfCsv("SPxxxx_DioLog.csv", "�蓮DIO����OK\n");
		printf("\n");
	}
	else{
		LogPrintfCsv("SPxxxx_DioLog.csv", "�蓮DIO����NG\n");
		printf("\n\a");
	}

	return bRetCont;
}

////////////////////////////////////////////
//SP_UPPER_IF_BOARD SAVENET�𗘗p����DIO����
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
	//DI���� (1�_�P�ʂ̓��͌���)
	/////////////////////////////////////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n\n");
		printf("-----------------------------------------------\n");
		printf("SP_UPPER_IF_BOARD: DIO�����������J�n���܂����B\n");
		for( ii=TEST_PATTERN0; ii<(TEST_PATTERN0 + UPPER1_TEST_PATTERN_NUM); ii++ ){
			bZeroChkEnable = TRUE;
			if( ii == TEST_PATTERN2 || ii == TEST_PATTERN8 ){
				bZeroChkEnable = FALSE;	//�p�^�[��8�ł͕���ON���͂����邽�ߑ�Adr��Zero�`�F�b�N�͂��Ȃ�
			}
			else{
				bZeroChkEnable = TRUE;
			}

			if( (m_wUpper1TestPattern[ii].wInAdr >= SP_XXXX_SW1_START_ADR+3) &&
				(m_wUpper1TestPattern[ii].wInAdr <= SP_XXXX_SW1_START_ADR+8) ){
				wAnalogBitMask = 0xc000;	//�A�i���O���̓A�h���X�͉���14bit��0�Ń}�X�N����
			}
			else{
				wAnalogBitMask = DIO_TEST_ALL_BIT;
			}

			if( m_wUpper1TestPattern[ii].wTestBitNum > 0 ){
				if( bRetCont == TRUE ){
					bRetCont = DioAutoTestSelPattern(m_wUpper1TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);
				}
				else{
					DioAutoTestSelPattern(m_wUpper1TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);	//NG���������������������𑱍s
					//break;
				}
			}
		}
	}

	return bRetCont;
}

////////////////////////////////////////////
//SP_UPPER_IF_BOARD2 SAVENET�𗘗p����DIO����
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
	//HLS�ʐM��~���̏o��OFF�e�X�g
	printf("\n");
	printf("-----------------------------------------------\n");
	printf(" HLS�ʐM��~���̏o��OFF�m�F\n");
	printf("-----------------------------------------------\n");
	/////////////////////////////////////////////////////////////////////
	//if( bRetCont != TRUE ){
	//	printf("\nHLS�ʐM��~���̏o��OFF�e�X�g�����{���܂����H(y or n) -->");
	//	nInputVal = KeyInputWait();
	//	if( nInputVal != 'y' && nInputVal != 'Y' && nInputVal != 0x0d ){
	//		printf(" n\n");
	//		bHlsStopTest = FALSE;	//HLS�ʐM��~���̏o��OFF���������{���Ȃ�
	//	}
	//	else{
	//		printf(" y\n");
	//	}
	//}
	//if( bHlsStopTest == TRUE ){
		//CN71 OUT3_0�`OUT3_7 �o��OFF�e�X�g
		memset(wTestPatten, 0, sizeof(wTestPatten));
		wTestPatten[nPatternCnt++] = TEST_PATTERN9;
		while(1){
			if( HlsOutStopTest(m_wUpper2TestPattern, wTestPatten, nPatternCnt, 300) != TRUE ){
				printf("\n�����𒆎~���܂����H(y or n) -->");
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
	//DI���� (1�_�P�ʂ̓��͌���)
	/////////////////////////////////////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n\n");
		printf("-----------------------------------------------\n");
		printf("SP_UPPER_IF_BOARD2: DIO�����������J�n���܂����B\n");
		for( ii=TEST_PATTERN0; ii<TEST_PATTERN0+UPPER2_TEST_PATTERN_NUM; ii++ ){
			if( ii == TEST_PATTERN2 || ii == TEST_PATTERN8 /*|| ii == TEST_PATTERN9 || ii == TEST_PATTERN15 || ii == TEST_PATTERN16*/){
				bZeroChkEnable = FALSE;	//�p�^�[��2, 8�ł͕���ON���͂����邽�ߑ�Adr��Zero�`�F�b�N�͂��Ȃ�
			}
			else{
				bZeroChkEnable = TRUE;
			}

			if( (m_wUpper2TestPattern[ii].wInAdr >= SP_XXXX_SW1_START_ADR+3) &&
				(m_wUpper2TestPattern[ii].wInAdr <= SP_XXXX_SW1_START_ADR+8) ){
				wAnalogBitMask = 0xc000;	//�A�i���O���̓A�h���X�͉���14bit��0�Ń}�X�N����
			}
			else{
				wAnalogBitMask = DIO_TEST_ALL_BIT;
			}

			if( m_wUpper2TestPattern[ii].wTestBitNum > 0 ){
				if( bRetCont == TRUE ){
					bRetCont = DioAutoTestSelPattern(m_wUpper2TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);
				}
				else{
					DioAutoTestSelPattern(m_wUpper2TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);	//NG���������������������𑱍s
					//break;
				}
			}
		}
	}

	return bRetCont;
}

/***
///////////////////////////////////////////////
// SP_UPPER_IF_BOARD2 �A���[�����̓e�X�g�i���g�p�j
BOOL	UpperAlarmInOutTest(WORD wOutAdr1, WORD wOutData1, WORD wOutAdr2, WORD wOutData2, 
							WORD wInAdr1, WORD wChkData1, WORD wInAdr2, WORD wChkData2, DWORD dwChkTime)
{
	DWORD	dwStartTick;
	DWORD	dwInData1, dwInData2;
	BOOL	bRet = TRUE;
	
	//�e�X�g�f�[�^���o�͂���
	DATAout(BoardId, wOutAdr1, wOutData1);
	DATAout(BoardId, wOutAdr2, wOutData2);

	Sleep_Cnt(50);

	//���͒l���������擾�ł��邱�Ƃ��m�F����
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
//SP_FRONT_IF_BOARD2 SAVENET�𗘗p����DIO����
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
	//HLS�ʐM��~���̏o��OFF�e�X�g
	/////////////////////////////////////////////////////////////////////
	printf("\n");
	printf("-----------------------------------------------\n");
	printf(" HLS�ʐM��~���̏o��OFF�m�F\n");
	printf("-----------------------------------------------\n");
	//if( bRetCont != TRUE ){
	//	printf("\nHLS�ʐM��~���̏o��OFF�e�X�g�����{���܂����H(y or n) -->");
	//	nInputVal = KeyInputWait();
	//	if( nInputVal != 'y' && nInputVal != 'Y' && nInputVal != 0x0d ){
	//		printf(" n\n");
	//		bHlsStopTest = FALSE;	//HLS�ʐM��~���̏o��OFF���������{���Ȃ�
	//	}
	//	else{
	//		printf(" y\n");
	//	}
	//}
	//if( bHlsStopTest == TRUE ){
		memset(wTestPatten, 0, sizeof(wTestPatten));
		//CN61 OUT1_0�`OUT1_f �o��OFF�e�X�g
		wTestPatten[nPatternCnt++] = TEST_PATTERN10;
		//CN33 OUT2_0�`OUT2_3 �o��OFF�e�X�g
		wTestPatten[nPatternCnt++] = TEST_PATTERN15;
		//CN41 OUT3_0�`OUT3_b �o��OFF�e�X�g
		wTestPatten[nPatternCnt++] = TEST_PATTERN16;
		//CN42 OUT3_0�`OUT3_b �o��OFF�e�X�g
		wTestPatten[nPatternCnt++] = TEST_PATTERN18;

		while(1){
			if( HlsOutStopTest(m_wFront2TestPattern, wTestPatten, nPatternCnt, 300) != TRUE ){
				printf("\n�����𒆎~���܂����H(y or n) -->");
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
	//DI���� (1�_�P�ʂ̓��͌���)
	/////////////////////////////////////////////////////////////////////
	if( bRetCont == TRUE ){
		printf("\n\n");
		printf("-----------------------------------------------\n");
		printf("SP_FRONT_IF_BOARD2: DIO�����������J�n���܂����B\n");
		for( ii=TEST_PATTERN0; ii<TEST_PATTERN0+FRONT_TEST_PATTERN_NUM; ii++ ){
			if( ii == TEST_PATTERN2 || ii == TEST_PATTERN8 || ii == TEST_PATTERN9 || ii == TEST_PATTERN15 || ii == TEST_PATTERN16){
				bZeroChkEnable = FALSE;	//�p�^�[��2, 8, 9, 15, 16�ł͕���ON���͂����邽�ߑ�Adr��Zero�`�F�b�N�͂��Ȃ�
			}
			else{
				bZeroChkEnable = TRUE;
			}

			if( (m_wFront2TestPattern[ii].wInAdr >= SP_XXXX_SW1_START_ADR+3) &&
				(m_wFront2TestPattern[ii].wInAdr <= SP_XXXX_SW1_START_ADR+6) ){
				wAnalogBitMask = 0xc000;	//�A�i���O���̓A�h���X�͉���14bit��0�Ń}�X�N����
			}
			else{
				wAnalogBitMask = DIO_TEST_ALL_BIT;
			}
			
			if( bRetCont == TRUE ){
				bRetCont = DioAutoTestSelPattern(m_wFront2TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);
				if( bRetCont != TRUE ){
					printf("\n�����𒆎~���܂����H(y or n) -->");
					nInputVal = KeyInputWaitYorN();
					if( nInputVal == 'y' ){
						break;
					}
				}
			}
			else{
				DioAutoTestSelPattern(m_wFront2TestPattern, (WORD)ii, 100, bZeroChkEnable, wAnalogBitMask);	//NG���������������������𑱍s
				//break;
			}
		}
	}
	return bRetCont;
}

///////////////////////////////////////////////
// DIO�܂�Ԃ��e�X�g
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

		//���͒l���������擾�ł��邱�Ƃ��m�F����
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, wSnInAdr, (LPINT*)&dwIndata);
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//�f�[�^�s��v�G���[
				LogPrintfCsv("SPxxxx_DioLog.csv",
					" Out_Adr%d-b%d(0x%04X):In_Adr%d-b%d(0x%04d) [NG:�f�[�^�s��v]\n", 
						wSnOutAdr, wOutBit, wOutVal, wSnInAdr, wInBit, (int)dwIndata);
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);

		if( bRet != TRUE ){
			break;
		}
	}

	//�S�_ON�m�F
	if( bRet == TRUE ){
		wOutVal = 0x0000;
		wChkVal = 0x0000;
		for( wOutBit=wSnOutStart, wInBit=wSnInStart; wOutBit<=wSnOutEnd; wOutBit++, wInBit++ ){
			wOutVal = wOutVal | (0x0001 << wOutBit);
			wChkVal = wChkVal | (0x0001 << wInBit);
		}
		DATAout(BoardId, wSnOutAdr, wOutVal);
		Sleep_Cnt(50);

		//���͒l���������擾�ł��邱�Ƃ��m�F����
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, wSnInAdr, (LPINT*)&dwIndata);
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//�f�[�^�s��v�G���[
				LogPrintfCsv("SPxxxx_DioLog.csv",
					" Out_Adr%d-b%d(0x%04X):In_Adr%d-b%d(0x%04d) [NG:�f�[�^�s��v]\n", 
						wSnOutAdr, wOutBit, wOutVal, wSnInAdr, wInBit, (int)dwIndata);
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);
	}

	//�S�_OFF�m�F
	if( bRet == TRUE ){
		wOutVal = 0x0000;
		wChkVal = 0x0000;
		DATAout(BoardId, wSnOutAdr, wOutVal);
		Sleep_Cnt(50);

		//���͒l���������擾�ł��邱�Ƃ��m�F����
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, wSnInAdr, (LPINT*)&dwIndata);
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//�f�[�^�s��v�G���[
				LogPrintfCsv("SPxxxx_DioLog.csv",
					" Out_Adr%d-b%d(0x%04X):In_Adr%d-b%d(0x%04d) [NG:�f�[�^�s��v]\n", 
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
// DIO ���A�h���X�̓��͂�ON/OFF�ω����Ȃ����Ƃ��m�F����
BOOL OtherUnitInputZeroChk( WORD wStartAdr, WORD wEndAdr, WORD wSkipAdr )
{
	DWORD	termerr;            // �^�[�~�i���ʐM���
	DWORD	dwIndata;
	WORD	ii;
	BOOL	bRet = TRUE;

	for( ii=wStartAdr; ii<=wEndAdr; ii++ ){
		if(ii != wSkipAdr){
			DATAin(BoardId, ii, (LPINT*)&dwIndata);
			if( dwIndata != 0 ){
				bRet = FALSE;
				LogPrintfCsv("SPxxxx_DioLog.csv",
							"HLS Adr.%d ���A�h���X�f�[�^�O�`�F�b�N�G���[(0x%04X)\n", ii, (WORD)dwIndata);
			}

			termerr = 0xffff;
			TERMchk(BoardId, ii, (LPINT*)&termerr);
			if(termerr & 0x7C00){
				bRet = FALSE;		//HLS�ʐM�G���[
				LogPrintfCsv("SPxxxx_DioLog.csv",
							"HLS Adr.%d �ʐM�G���[(0x%04X)\n", ii, termerr);
			}
		}
	}
	
	return bRet;
}

/////////////////////////////////////////////////////////////////////
//HLS�ʐM��~���̏o��OFF�e�X�g
BOOL HlsOutStopTest( DIO_AUTO_TEST_TABLE *iotbl, WORD *wPatternNo, int nPatternCnt, DWORD dwChkTime )
{
	BOOL	bRet=TRUE;
	WORD	ii, jj;
	WORD	wOutVal[16];
	WORD	wChkVal[16];
	DWORD	dwIndata[16];
	DWORD	dwStartTick;
	DWORD	OutTermChk;		// �o�͑��^�[�~�i���ʐM���
	DWORD	InTermChk;		// ���͑��^�[�~�i���ʐM���
	DWORD	OutCmpVal;
	DWORD	InCmpVal;
	int		nInputVal;
	BOOL	bChkFlg;

	//�O�񌟍��ŃG���[�̂Ƃ������J�n���ɏo��ON�̂܂܂ɂȂ�Ƃ�������̂ŁA�o�͒l��S�N���A
	SnOutputInit(FALSE);

	//(�O����)
	//SP����̏o�͂�ON���邱�ƂŁASAVENET�܂�Ԃ����͒l��ON���邱�Ƃ��m�F����
	if( bRet == TRUE ){
		for( jj=0; jj<nPatternCnt; jj++ ){
			wOutVal[jj] = 0x0000;
			wChkVal[jj] = 0x0000;
			for( ii=0; ii<iotbl[wPatternNo[jj]].wTestBitNum; ii++ ){
				wOutVal[jj] = wOutVal[jj] | (0x0001 << iotbl[wPatternNo[jj]].wOutBit[ii]);
				wChkVal[jj] = wChkVal[jj] | (0x0001 << iotbl[wPatternNo[jj]].wInBit[ii]);
			}
			DATAout(BoardId, iotbl[wPatternNo[jj]].wOutAdr, wOutVal[jj]);	//SP����̏o�͂�ON����
		}
		Sleep_Cnt(50);	//��������wait

		//���͒l�����肵�ēǂݏo���邱�Ƃ��m�F����
		dwStartTick = GetTickCount();
		do{
			for( jj=0; jj<nPatternCnt; jj++ ){
				DATAin(BoardId, iotbl[wPatternNo[jj]].wInAdr, (LPINT*)&dwIndata[jj]);
				if( (WORD)dwIndata[jj] != wChkVal[jj] ){
					bRet = FALSE;		//�f�[�^�s��v�G���[
					LogPrintfCsv("SPxxxx_DioLog.csv",
						"[HLS STOP TestPtn%d_1]  Out_Adr%d=0x%04X��In_Adr%d=0x%04X [(�S�_ON�e�X�gNG:)�f�[�^�s��v]\n", 
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
						bRet = FALSE;		//HLS�ʐM�G���[
						LogPrintfCsv("SPxxxx_DioLog.csv",
									"[HLS STOP TestPtn%d] HLS Adr.%d �ʐM�G���[(0x%04X)\n", wPatternNo[jj], iotbl[wPatternNo[jj]].wOutAdr, OutTermChk);
						break;
					}

					InTermChk = 0xffff;
					TERMchk(BoardId, (WORD)iotbl[wPatternNo[jj]].wInAdr, (LPINT*)&InTermChk);
					if(InTermChk & 0x7C00){
						bRet = FALSE;		//HLS�ʐM�G���[
						LogPrintfCsv("SPxxxx_DioLog.csv",
									"[HLS STOP TestPtn%d] HLS Adr.%d �ʐM�G���[(0x%04X)\n", wPatternNo[jj], iotbl[wPatternNo[jj]].wInAdr, InTermChk);
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

	//HLS�ʐM��~���ƒʐM���펞�̓��͒l���Ď�
	for( ii=0; ii<2; ii++ ){		//ii=0:HLS�ʐM��~���̏o��OFF�m�F�A1:HLS�ʐM���펞�̏o��ON�m�F
		printf("\nSP�����HLS�ʐM��%s���Ă�������\a\n", ((ii==0) ? "��~":"����"));
		if( ii == 0 ){
			OutCmpVal = 0x7C00;		//�o�͑��͒ʐM�ُ�ɂȂ�̂�҂�
			InCmpVal = 0x0000;		//���͑��͒ʐM����ɂȂ�̂�҂�
		}
		else{
			OutCmpVal = 0x0000;		//�o�͑��͒ʐM����ɂȂ�̂�҂�
			InCmpVal = 0x0000;		//���͑��͒ʐM����ɂȂ�̂�҂�
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
							printf("\rHLS��~�҂�...Out(Adr%d=%04X),In(Adr%d=%04X)\r",
									iotbl[wPatternNo[jj]].wOutAdr, wOutVal[jj],
									iotbl[wPatternNo[jj]].wInAdr, (WORD)dwIndata[jj]);
							bChkFlg = FALSE;	//HLS��~��ɏo�͂�OFF���Ă��Ȃ�
							break;
						}
					}
					else{
						if( (WORD)dwIndata[jj] != wChkVal[jj] ){
							printf("\rHLS�����҂�...Out(Adr%d=%04X),In(Adr%d=%04X)\r",
									iotbl[wPatternNo[jj]].wOutAdr, wOutVal[jj],
									iotbl[wPatternNo[jj]].wInAdr, (WORD)dwIndata[jj]);
							bChkFlg = FALSE;	//HLS�ʐM������ɏo�͂�ON���Ă��Ȃ�
							break;
						}
					}
				}
				else{
					printf("\rHLS�ʐM%s�҂�...\r",((ii==0) ? "��~":"����"));
					bChkFlg = FALSE;			//SP����HLS�ʐM��(ii=0:��~,1:����)���Ă��Ȃ�
				}

				if( _kbhit() ){
					nInputVal = _getch();
					if( nInputVal == 0x1B ){
						bRet = FALSE;			//���f���삠��
						printf("\n");
						LogPrintfCsv("SPxxxx_DioLog.csv", "[HLS STOP TestPtn%d] ���f����܂���\n", wPatternNo[jj]);
						break;
					}
				}
			}
			if( bChkFlg == TRUE || bRet != TRUE ){
				printf("\n");
				break;		//SP���̒ʐM(ii=0:��~,1:����)�����o
			}
		}	//while(1)�̏I���

		Sleep_Cnt(50);

		//////////////////////////////////////////////////////////////
		//���͒l(ii=0:OFF,1:ON)�����肵�ēǂݏo���邱�Ƃ��m�F����
		if( bRet == TRUE ){
			dwStartTick = GetTickCount();
			do{
				for( jj=0; jj<nPatternCnt; jj++ ){
					if( ii == 0 ){
						InCmpVal = 0x0000;				//HLS��~���͓��͂�OFF�ł��邱�Ƃ��m�F����
					}
					else{
						InCmpVal = (DWORD)wChkVal[jj];	//HLS����ʐM���͓��͂�ON�ł��邱�Ƃ��m�F����
					}
					
					DATAin(BoardId, iotbl[wPatternNo[jj]].wInAdr, (LPINT*)&dwIndata[jj]);
					if( dwIndata[jj] != InCmpVal ){
						bRet = FALSE;		//�f�[�^(ii=0:OFF,1:ON)���̓G���[
						LogPrintfCsv("SPxxxx_DioLog.csv",
							"[HLS STOP TestPtn%d_3]  Out_Adr%d=0x%04X��In_Adr%d=0x%04X [NG: HLS%s���̓��̓f�[�^�ُ�]\n", 
								wPatternNo[jj],
								iotbl[wPatternNo[jj]].wOutAdr,
								wOutVal[jj],
								iotbl[wPatternNo[jj]].wInAdr,
								(int)dwIndata[jj],
								((ii==0) ? "��~":"����"));
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
	}	//for(ii=0:HLS�ʐM��~���̏o��OFF�m�F�A1:HLS�ʐM���펞�̏o��ON�m�F)�̏I���

	//�㏈��
	for( jj=0; jj<nPatternCnt; jj++ ){
		DATAout(BoardId, iotbl[wPatternNo[jj]].wOutAdr, 0x0000);	//�o�͂�OFF����
	}

	if( bRet == TRUE ){
		LogPrintfCsv("SPxxxx_DioLog.csv", "HLS�ʐM��~���̏o��OFF�m�F <OK>\n");
	}
	else{
		LogPrintfCsv("SPxxxx_DioLog.csv", "HLS�ʐM��~���̏o��OFF�m�F <NG!>\n");
	}
	return bRet;
}

///////////////////////////////////////////////
// DIO�܂�Ԃ��e�X�g(����bit)
BOOL DioAutoTestSelPattern( DIO_AUTO_TEST_TABLE *iotbl, WORD wPatternNo, DWORD dwChkTime, BOOL bZeroChkEnable, WORD wInMaskBit )
{
	BOOL	bRet=TRUE;
	WORD	ii;
	//WORD	wOutBit;
	WORD	wOutVal;
	WORD	wChkVal;
	DWORD	dwIndata;
	DWORD	dwStartTick;
	DWORD	termerr;            // �^�[�~�i���ʐM���
	BOOL	bZeroChk;
	char	cSignalNameWork[2][16];
	char	cPinNameWork[2][16];
	
	WORD	wHlsZeroTestAdrStart_1 = 0;
	WORD	wHlsZeroTestAdrEnd_1 = 0;
	WORD	wHlsZeroTestAdrStart_2 = 0;
	WORD	wHlsZeroTestAdrEnd_2 = 0;

	if( m_nSPxxxxBoardID == SP_BID_FRONT_IF_BOARD ){
		//#7�`#10�̓A�i���O���͂ɂ�Zero�e�X�g�͂��Ȃ�
		wHlsZeroTestAdrStart_1 = SP_XXXX_SW1_START_ADR;		//#4
		wHlsZeroTestAdrEnd_1 = SP_XXXX_SW1_START_ADR+2;		//#6
		wHlsZeroTestAdrStart_2 = ADR_DI_48;
		wHlsZeroTestAdrEnd_2 = ADR_DI_58;
	}
	else if ( m_nSPxxxxBoardID == SP_BID_UPPER_IF_BOARD ){
		//#7�`#12�̓A�i���O���͂ɂ�Zero�e�X�g�͂��Ȃ�
		wHlsZeroTestAdrStart_1 = SP_XXXX_SW1_START_ADR;		//#4
		wHlsZeroTestAdrEnd_1 = SP_XXXX_SW1_START_ADR+2;		//#6
		wHlsZeroTestAdrStart_2 = ADR_DI_48;
		wHlsZeroTestAdrEnd_2 = ADR_DI_58;
	}
	else if ( m_nSPxxxxBoardID == SP_BID_MOTHER_BOARD ){
		if( m_nIniSpMotherBoardType == 0 ){		//SP_MOTHER_BOARD
			//#10�̓A�i���O���͂ɂ�Zero�e�X�g�͂��Ȃ�
			wHlsZeroTestAdrStart_1 = SP_XXXX_SW1_START_ADR;		//#4
			wHlsZeroTestAdrEnd_1 = SP_XXXX_SW2_START_ADR+1;		//#9
		}
		else{		//SPIN_MOTHER_BOARD
			//#9�̓A�i���O���͂ɂ�Zero�e�X�g�͂��Ȃ�
			wHlsZeroTestAdrStart_1 = SP_XXXX_SW1_START_ADR;		//#4
			wHlsZeroTestAdrEnd_1 = SP_XXXX_SW2_START_ADR+0;		//#8
		}
		wHlsZeroTestAdrStart_2 = ADR_DI_48;
		wHlsZeroTestAdrEnd_2 = ADR_DI_58;
	}

	//�O�񌟍��ŃG���[�̂Ƃ������J�n���ɏo��ON�̂܂܂ɂȂ�Ƃ�������̂ŁA�o�͒l��S�N���A
	SnOutputInit(FALSE);
	//Sleep_Cnt(50);
	
	//bit walk test
	for( ii=0; ii<iotbl[wPatternNo].wTestBitNum; ii++ ){
	//for( wOutBit=0; wOutBit<=15; wOutBit++ ){
		wOutVal = 0x0001 << iotbl[wPatternNo].wOutBit[ii];
		wChkVal = 0x0001 << iotbl[wPatternNo].wInBit[ii];
		DATAout(BoardId, iotbl[wPatternNo].wOutAdr, wOutVal);
		Sleep_Cnt(50);

		//���͒l���������擾�ł��邱�Ƃ��m�F����
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, iotbl[wPatternNo].wInAdr, (LPINT*)&dwIndata);
			dwIndata = dwIndata & (DWORD)wInMaskBit;	//�A�i���O�f�[�^�r�b�g��0�Ń}�X�N����
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//�f�[�^�s��v�G���[

				//�o�͑�MIL�R�l�N�^�ԍ��𕶎���Ŏ擾
				GetSpInternalSignalName(iotbl[wPatternNo].wOutAdr, iotbl[wPatternNo].wOutBit[ii], "Out");
				strcpy(&cSignalNameWork[0][0], m_cSpInternalSignalName);

				//�o�͑��M�������擾
				GetPinNumMilTerm2(iotbl[wPatternNo].wOutAdr, iotbl[wPatternNo].wOutBit[ii]);
				strcpy(&cPinNameWork[0][0], m_cMilConnectorPinName);
				
				//���͑�MIL�R�l�N�^�ԍ��𕶎���Ŏ擾
				GetSpInternalSignalName(iotbl[wPatternNo].wInAdr, iotbl[wPatternNo].wInBit[ii], "In");
				strcpy(&cSignalNameWork[1][0], m_cSpInternalSignalName);

				//���͑��M�������擾
				GetPinNumMilTerm2(iotbl[wPatternNo].wInAdr, iotbl[wPatternNo].wInBit[ii]);
				strcpy(&cPinNameWork[1][0], m_cMilConnectorPinName);

				LogPrintfCsv("SPxxxx_DioLog.csv",
					"[TestPtn%d] Out_Adr%d%s=0x%04X(Bit%d%s)��In_Adr%d%s=0x%04X(Bit%d%s) [(Walking�e�X�gNG:)�f�[�^�s��v]\n", 
						wPatternNo,
						iotbl[wPatternNo].wOutAdr,
						&cSignalNameWork[0][0],			//�o�͑�MIL�R�l�N�^�ԍ�
						wOutVal,
						iotbl[wPatternNo].wOutBit[ii],	//�o�͑��M����
						&cPinNameWork[0][0],

						iotbl[wPatternNo].wInAdr,
						&cSignalNameWork[1][0],			//���͑�MIL�R�l�N�^�ԍ�
						(int)dwIndata,
						iotbl[wPatternNo].wInBit[ii],
						&cPinNameWork[1][0]);			//���͑��M����
			}
			else{
				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wOutAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS�ʐM�G���[
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d �ʐM�G���[(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wOutAdr, termerr);
				}

				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wInAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS�ʐM�G���[
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d �ʐM�G���[(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wInAdr, termerr);
				}
			}

			if( bRet == TRUE && bZeroChkEnable == TRUE){
				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_1, wHlsZeroTestAdrEnd_1, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zero�`�F�b�N�G���[�i0�ȊO�̓��͂����o�j
				}

				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_2, wHlsZeroTestAdrEnd_2, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zero�`�F�b�N�G���[�i0�ȊO�̓��͂����o�j
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

	//�S�_ON�m�F
	if( bRet == TRUE ){
		wOutVal = 0x0000;
		wChkVal = 0x0000;
		for( ii=0; ii<iotbl[wPatternNo].wTestBitNum; ii++ ){
			wOutVal = wOutVal | (0x0001 << iotbl[wPatternNo].wOutBit[ii]);
			wChkVal = wChkVal | (0x0001 << iotbl[wPatternNo].wInBit[ii]);
		}
		DATAout(BoardId, iotbl[wPatternNo].wOutAdr, wOutVal);
		Sleep_Cnt(50);

		//���͒l���������擾�ł��邱�Ƃ��m�F����
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, iotbl[wPatternNo].wInAdr, (LPINT*)&dwIndata);
			dwIndata = dwIndata & (DWORD)wInMaskBit;	//�A�i���O�f�[�^�r�b�g��0�Ń}�X�N����
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//�f�[�^�s��v�G���[
				LogPrintfCsv("SPxxxx_DioLog.csv",
					"[TestPtn%d]  Out_Adr%d=0x%04X��In_Adr%d=0x%04X [(�SON�e�X�gNG:)�f�[�^�s��v]\n", 
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
					bRet = FALSE;		//HLS�ʐM�G���[
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d �ʐM�G���[(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wOutAdr, termerr);
				}

				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wInAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS�ʐM�G���[
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d �ʐM�G���[(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wInAdr, termerr);
				}
			}

			if( bRet == TRUE && bZeroChkEnable == TRUE){
				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_1, wHlsZeroTestAdrEnd_1, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zero�`�F�b�N�G���[�i0�ȊO�̓��͂����o�j
				}

				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_2, wHlsZeroTestAdrEnd_2, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zero�`�F�b�N�G���[�i0�ȊO�̓��͂����o�j
				}
			}
			if( bRet != TRUE ){
				break;
			}
			Sleep_Cnt(5);
		}while((DWORD)(GetTickCount()-dwStartTick) < dwChkTime);
	}

	//�S�_OFF�m�F
	if( bRet == TRUE ){
		wOutVal = 0x0000;
		wChkVal = 0x0000;
		DATAout(BoardId, iotbl[wPatternNo].wOutAdr, wOutVal);
		Sleep_Cnt(50);

		//���͒l���������擾�ł��邱�Ƃ��m�F����
		dwStartTick = GetTickCount();
		do{
			DATAin(BoardId, iotbl[wPatternNo].wInAdr, (LPINT*)&dwIndata);
			dwIndata = dwIndata & (DWORD)wInMaskBit;	//�A�i���O�f�[�^�r�b�g��0�Ń}�X�N����
			if( (WORD)dwIndata != wChkVal ){
				bRet = FALSE;		//�f�[�^�s��v�G���[
				LogPrintfCsv("SPxxxx_DioLog.csv",
					"[TestPtn%d]  Out_Adr%d=0x%04X��In_Adr%d=0x%04X [(�SOFF�e�X�gNG:)�f�[�^�s��v]\n", 
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
					bRet = FALSE;		//HLS�ʐM�G���[
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d �ʐM�G���[(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wOutAdr, termerr);
				}

				termerr = 0xffff;
				TERMchk(BoardId, (WORD)iotbl[wPatternNo].wInAdr, (LPINT*)&termerr);
				if(termerr & 0x7C00){
					bRet = FALSE;		//HLS�ʐM�G���[
					LogPrintfCsv("SPxxxx_DioLog.csv",
								"[TestPtn%d] HLS Adr.%d �ʐM�G���[(0x%04X)\n", wPatternNo, iotbl[wPatternNo].wInAdr, termerr);
				}
			}
			if( bRet == TRUE && bZeroChkEnable == TRUE){
				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_1, wHlsZeroTestAdrEnd_1, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zero�`�F�b�N�G���[�i0�ȊO�̓��͂����o�j
				}

				bZeroChk = OtherUnitInputZeroChk(wHlsZeroTestAdrStart_2, wHlsZeroTestAdrEnd_2, iotbl[wPatternNo].wInAdr);
				if(bZeroChk != TRUE){
					bRet = FALSE;		//Zero�`�F�b�N�G���[�i0�ȊO�̓��͂����o�j
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
// DIO�܂�Ԃ��e�X�g (1bit�̂�)
#define LPBACK_READ_NUM		32		//�ǂݏo���J��Ԃ���
BOOL DioLoopBackTest( WORD wOutAdr, WORD wOutBit, WORD wInAdr, WORD wInBit )
{
	DWORD	dwInData;
	DWORD	dwOutData;
	int		ii;
	BOOL	berr = TRUE;
	int		nInputVal;

	if( BoardId != 0xffff ){
		//�܂�Ԃ�OFF���͂��m�F����
		dwOutData = (DWORD)(0x0001 << wInBit);
		DATAout(BoardId, wOutAdr, (WORD)dwOutData);
		Sleep_Cnt(100);
		printf("\n");

		//�܂�Ԃ����͒l��ON�ɂȂ�܂�WAIT����(SW����҂�)
		while(ForeverLoop){
			Sleep_Cnt(10);
			if( _kbhit() ){
				nInputVal = _getch();
				if( nInputVal == 0x1B ){
					berr = FALSE;
					printf("\a�܂�Ԃ����̓e�X�g���L�����Z������܂���\a\n");
					break;
				}
			}

			dwInData = dwInData & (0x0001 << wInBit);
			DATAin(BoardId, wInAdr, (LPINT*)&dwInData);
			if( dwInData == dwOutData ){	//�f�[�^�̐܂�Ԃ�����OK
				//printf("ON�܂�Ԃ�����OK                 \n");
				break;
			}
			else{
				printf("\r�܂�Ԃ����͑҂�... (Esc:���~)\r");
			}
		}
		
		//���肵�ē��͒l���ǂݏo���邱�Ƃ��m�F����		
		for( ii=0; ii<LPBACK_READ_NUM; ii++ ){	//32��
			Sleep_Cnt(5);
			DATAin(BoardId, wInAdr, (LPINT*)&dwInData);
			dwInData = dwInData & (0x0001 << wInBit);
			if( dwInData != dwOutData ){
				berr = FALSE;
			}
		}

		//�܂�Ԃ�OFF���͂��m�F����
		if( berr == TRUE ){
			dwOutData = 0;
			DATAout(BoardId, wOutAdr, (WORD)dwOutData);
			Sleep_Cnt(100);
			for( ii=0; ii<LPBACK_READ_NUM; ii++ ){	//32��
				Sleep_Cnt(5);
				DATAin(BoardId, wInAdr, (LPINT*)&dwInData);
				dwInData = dwInData & (0x0001 << wInBit);
				if( dwInData != dwOutData ){
					berr = FALSE;
				}
			}

			printf("[%s]Adr%d-B%d �� Adr%d-A%d�܂�Ԃ�����          \n",
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
//D/A�o�͐��x�m�F(SP_XXXX_BOARD�p)
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
		"������ [%s] D/A�o�͐��x�m�F %s %s START ������\n", m_cSPxxxxBoardName, strDate, strTime);

	////SP_XXXX �Ɠ���|�[�g���g���̂ŁA�ڑ�����UCLOSE����
	//if( m_hSpComHandle != INVALID_HANDLE_VALUE ){
	//	CloseHandle(m_hSpComHandle);
	//	m_hSpComHandle = INVALID_HANDLE_VALUE;
	//}

	for( nChNum=0; nChNum<SP_XXXX_DACH_MAX; nChNum++ ){
		printf("\n------------------------------------------------\n");
		printf("�@�}���`���[�^�̃v���[�u��DA ch%d�ɐڑ����Ă��������B\n", nChNum);
		printf("�@RS-232C�ڑ����}���`���[�^�ɕύX���Ă��������B\n");
		printf("\n------------------------------------------------\n");
		printf("�@SAVENET(6M/H)�ʐM�ڑ����s���Ă��邱�Ƃ��m�F���Ă��������B\n");
		printf("--------------------------------------------------\n");
		printf("�������ł����牽���L�[�������Ă��������B\n\n");
		nInputVal = KeyInputWait();
		if ( nInputVal == 0x1B ){
			nExitFlg = 1;
			bRetCont = FALSE;
			break;
		}

		if( nChNum == 0 ){
			//�}���`���[�^�Ōv�����邽�߂̏���
			retStart_HP34401A = Start_HP34401A(
				m_cComPort,
				&forMultiMaterHandles	//�ʐM���\�[�X�̃n���h����Ԃ��A�h���X
			);
			if( ! retStart_HP34401A ){
				LogPrintf("MultiMeter 34401A Initialize Error!!\n");
				nExitFlg = 1;
				bRetCont = FALSE;
				break;
			}

			//�}���`���[�^�̃����W��d�����͂ɐݒ肷��
			retChangeVA_DC_HP34401A = ChangeVoltageDC_HP34401A(
				forMultiMaterHandles	//�ʐM���\�[�X�n���h��
			);
			if( ! retChangeVA_DC_HP34401A ){
				LogPrintf("MultiMeter 34401A VoltageSetting Error!!\n");
				End_HP34401A(forMultiMaterHandles);	//�ʐM���\�[�X�̃n���h�������
				nExitFlg = 1;
				bRetCont = FALSE;
				break;
			}
		}

		wTermAdr = (unsigned short)(nChNum+(SP_XXXX_SW2_START_ADR+2));
		//
		//[DA] 0.00V(0x000), 0.01V(0x021), 2.50V(0x2000), 4.99V(0x3FDE), 5.00V(0x3FFF) ���}���`���[�^�ŃT���v�����O����
		//
		for( ii=0; ii<DA_OUTPUT_RANGE_CHK_NUM; ii++ ){
			//SAVENET����DA�o�͒l��ݒ肷��
			err = DATAout(BoardId, wTermAdr, m_wDaOutputStdVal[ii]);
			if( err != SNMA_NO_ERROR ){
				printf("DATAout() err=%ld\a\n", err);
				HitAnyKeyWait();
				nExitFlg = 1;
				bRetCont = FALSE;
				break;
			}
			Sleep_Cnt(1000);
			
			//�}���`���[�^����DA�o�͒l���T���v�����O����
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

			//���ʏo��
			if( bRetCont == TRUE && nExitFlg == 0 ){
				bTestResult = TRUE;

				GetDaMinMaxAve(fDADatDim, &fDADatMin, &fDADatMax, &fDADatAve);

				printf("\n");
				LogPrintfCsv("SPxxxx_DaLog.csv", "----<(D/A ch%d) %5.3fV ���x�m�F>---\n", nChNum, m_fDaOutputStdVoltage[ii]);
				LogPrintfCsv("SPxxxx_DaLog.csv", "CheckType, Voltafge(V), Precision(%%), Result\n");

				//MIN
				fCalcVal = (double)(fabs((double)m_fDaOutputStdVoltage[ii] - fDADatMin) * 100.0) / (double)(5.0);
				LogPrintfCsv("SPxxxx_DaLog.csv", "MIN, %f, %f, %s\n",
					fDADatMin, fCalcVal, (fCalcVal > DA_OK_RATE) ? "NG": "OK");
				printf("(ch%d)%5.3fV DA���x(MIN)=%5.3f%%\n\n", nChNum, m_fDaOutputStdVoltage[ii], fCalcVal);
				if( fCalcVal > DA_OK_RATE ){
				//	nResult[nChNum-1][jj][kk] = FALSE;
					bTestResult = FALSE;
				}

				//MAX
				fCalcVal = (double)(fabs((double)m_fDaOutputStdVoltage[ii] - fDADatMax) * 100.0) / (double)(5.0);
				LogPrintfCsv("SPxxxx_DaLog.csv", "MAX, %f, %f, %s\n",
					fDADatMax, fCalcVal, (fCalcVal > DA_OK_RATE) ? "NG": "OK");
				printf("(ch%d)%5.3fV DA���x(MAX)=%5.3f%%\n\n", nChNum, m_fDaOutputStdVoltage[ii], fCalcVal);
				if( fCalcVal > DA_OK_RATE ){
				//	nResult[nChNum-1][jj][kk] = FALSE;
					bTestResult = FALSE;
				}

				//AVE
				fCalcVal = (double)(fabs((double)m_fDaOutputStdVoltage[ii] - fDADatAve) * 100.0) / (double)(5.0);
				LogPrintfCsv("SPxxxx_DaLog.csv", "AVE, %f, %f, %s\n",
					fDADatAve, fCalcVal, (fCalcVal > DA_OK_RATE) ? "NG": "OK");
				printf("(ch%d)%5.3fV DA���x(AVE)=%5.3f%%\n\n", nChNum, m_fDaOutputStdVoltage[ii], fCalcVal);
				if( fCalcVal > DA_OK_RATE ){
				//	nResult[nChNum-1][jj][kk] = FALSE;
					bTestResult = FALSE;
				}

				if( bTestResult != TRUE ){
					printf("\n�������x��%5.2f%%�𒴂��܂���\a\a\n", DA_OK_RATE);
					printf("\n�����𒆎~���܂����H(y or n) -->");
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

	//�}���`���[�^�ʐM���\�[�X�̃n���h�������
	if( forMultiMaterHandles != NULL ){
		End_HP34401A(forMultiMaterHandles);
	}

	////RS232C�ڑ���SP_XXXX�ɖ߂�
	//if( m_hSpComHandle == INVALID_HANDLE_VALUE ){
	//	RS_SpComOpen();
	//	printf("\n------------------------------------------------\n");
	//	printf("�@RS-232C�ڑ���UPPER BOARD���ɖ߂��Ă��������B\n");
	//	printf("�@�������ł����牽���L�[�������Ă��������B\n\n");
	//	printf("\n------------------------------------------------\n");
	//	KeyInputWait();
	//}

	_strdate(&strDate[0]);
	_strtime(&strTime[0]);
	LogPrintfCsv("SPxxxx_DaLog.csv", 
		"������ [%s] D/A�o�͐��x�m�F %s %s FINISH ������\n", m_cSPxxxxBoardName, strDate, strTime);

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
		"������ [%s] DI(SIG)���͊m�F %s %s START ������\n", m_cSPxxxxBoardName, strDate, strTime);

	memset(dwInData, 0, sizeof(dwInData));
	memset(dwInData_old, 0, sizeof(dwInData_old));
	printf("--- Analog Connector DI���͊m�F ---\n");
	switch(m_nSPxxxxBoardID){
	case SP_BID_UPPER_IF_BOARD:
		nAdAdrOffs = 7;		//Adr7,10,11,12
		printf("[CN51(SIG1,SIG2)], [CN61�`CN63(DIN)]\n");
		break;
	case SP_BID_FRONT_IF_BOARD:
		nAdAdrOffs = 7;		//Adr7,8,9,10
		printf("[CN51�`CN52(SIG1)], [CN53�`CN54(SIG1,SIG2)]\n");
		break;
	}
	printf("Key Input (OK='Y', NG='N', ���~='Esc')\n\n");
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
				printf("���f���܂����B\a\n");
				LogPrintfCsv("SPxxxx_DiLog.csv", " DI(SIG)���͊m�F (���f)\n");
				break;
			}
			else if( nInputVal == 'y' || nInputVal == 'Y' || nInputVal == 0x0d ){
				LogPrintfCsv("SPxxxx_DiLog.csv", " DI(SIG)���͊m�F (OK)\n");
				break;
			}
			else if( nInputVal == 'n' || nInputVal == 'N' ){
				LogPrintfCsv("SPxxxx_DiLog.csv", " DI(SIG)���͊m�F (NG)\n");
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
		"������ [%s] DI(SIG)���͊m�F %s %s FINISH ������\n", m_cSPxxxxBoardName, strDate, strTime);
	return bRetCont;

}

///////////////////////////////////////////////////////
//SP_xxxx_BOARD��EEPROM�ɐݒ肳��Ă���{�[�hID���擾�i�܂��͕ύX�j����
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

	//�d����
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//�e�X�g���[�h
	m_cSendPacket[nSendSize++] = SP_RS_MODE_EEPSAVE;

	//�o�̓f�[�^�L���E�����t���O
	if( nNewBoardID == SP_BID_CHECK_ONLY ){
		cSaveEnable = '0';		//�`�F�b�N�̂ݎ��͖����t���O���Z�b�g����
	}
	else{
		cSaveEnable = '1';		//�{�[�hID�ύX���͗L���t���O���Z�b�g����
	}
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		m_cSendPacket[nSendSize++] = cSaveEnable;	//�{�[�hID�ύX����Bit15�̏�ԂɊ֌W����
	}

	//�o�̓f�[�^
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		sprintf((char*)&m_cSendPacket[nSendSize], "%04X", nNewBoardID);
		nSendSize+=4;
	}

	//�`�F�b�N�T���v�Z
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARD�ɑ΂��ăp�P�b�g���M�A�y�уf�[�^��M�҂�
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
			printf("��M�f�[�^�`�F�b�N�T���G���[(Hit any key)\a\n");
			KeyInputWait();
		}
	}
#ifndef	_RS_DEBUG_MODE
	else{
		printf("��M�f�[�^�T�C�Y(=%d)�G���[(Hit any key)\a\n", nRecvSize);
		KeyInputWait();
	}
#else
	nRecvBID = SP_BID_FRONT_IF_BOARD;
#endif
	return nRecvBID;
}

///////////////////////////////////////////////////////
//�A�i���O���͐��f�[�^���擾����
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

	//�d����
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//�e�X�g���[�h
	m_cSendPacket[nSendSize++] = SP_RS_MODE_RAW;

	//�o�̓f�[�^�L���E�����t���O
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		m_cSendPacket[nSendSize++] = '0';	//�S�Ė���
	}

	//�o�̓f�[�^
	for(ii=0; ii<(SP_XXXX_ADCH_MAX*4); ii++){
		m_cSendPacket[nSendSize++] = '0';
	}

	//�`�F�b�N�T���v�Z
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARD�ɑ΂��ăp�P�b�g���M�A�y�уf�[�^��M�҂�
	nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
	if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD ){
		ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
		ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
		if( ucCheckSumCalc == ucCheckSumRecv ){
			if( m_cRecvPacket[6+nch] == '1' ){	//���͒l�͗L���H
				memset(cTemp, 0, sizeof(cTemp));
				memcpy(cTemp, &m_cRecvPacket[12+4*nch], 4);
				nAdRawVal = (int)strtol(cTemp, &endptr, 16);
			}
			else{
				printf("���f�[�^���͕s��(Hit any key)\a\n");
				KeyInputWait();
			}
		}
		else{
			printf("��M�f�[�^�`�F�b�N�T���G���[(Hit any key)\a\n");
			KeyInputWait();
		}
	}
#ifndef	_RS_DEBUG_MODE
	else{
		printf("��M�f�[�^�T�C�Y(=%d)�G���[(Hit any key)\a\n", nRecvSize);
		KeyInputWait();
	}
#else
	nAdRawVal = 0x1000;
#endif
	return nAdRawVal;
}

////////////////////////////////////////
// �A�i���O���͕␳��̒l���X�V����
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

	//�d����
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//�e�X�g���[�h
	m_cSendPacket[nSendSize++] = SP_RS_MODE_NORMAL;

	//�o�̓f�[�^�L���E�����t���O
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		m_cSendPacket[nSendSize++] = '0';	//�S�Ė���
	}

	//�o�̓f�[�^
	for(ii=0; ii<(SP_XXXX_ADCH_MAX*4); ii++){
		m_cSendPacket[nSendSize++] = '0';
	}

	//�`�F�b�N�T���v�Z
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARD�ɑ΂��ăp�P�b�g���M�A�y�уf�[�^��M�҂�
	nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
	if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD  ){
		ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
		ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
		if( ucCheckSumCalc == ucCheckSumRecv ){
			if( m_cRecvPacket[6+nch] == '1' ){	//���͒l�͗L���H
				memset(cTemp, 0, sizeof(cTemp));
				memcpy(cTemp, &m_cRecvPacket[12+4*nch], 4);
				nAdCorrectVal = (int)strtol(cTemp, &endptr, 16);
			}
			else{
				printf("AD�␳��̃f�[�^���͕s��(Hit any key)\a\n");
				KeyInputWait();
			}
		}
		else{
			printf("��M�f�[�^�`�F�b�N�T���G���[(Hit any key)\a\n");
			KeyInputWait();
		}
	}
#ifndef	_RS_DEBUG_MODE
	else{
		printf("��M�f�[�^�T�C�Y(=%d)�G���[(Hit any key)\a\n", nRecvSize);
		KeyInputWait();
	}
#else
	nAdCorrectVal = 0x1000;
#endif
	return nAdCorrectVal;
}

////////////////////////////////////////
// �A�i���O���͕␳�l���X�V����
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

		//�d����
		sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
		nSendSize +=3;

		//�e�X�g���[�h
		m_cSendPacket[nSendSize++] = SP_RS_MODE_ADADJ;

		//�o�̓f�[�^�L���E�����t���O
		for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
			if( dwAdOffset[ii] == 0xffff || dwAdGain[ii] == 0xffff ){
				m_cSendPacket[nSendSize++] = '0';		//�X�V�Ȃ�
			}
			else{
				m_cSendPacket[nSendSize++] = '1';		//�X�V����
				dwUpdateFlg[ii] = 0x8000;	//�X�V����
			}
		}

		//�o�̓f�[�^
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

		//�`�F�b�N�T���v�Z
		ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
		sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
		nSendSize+=2;

		m_cSendPacket[nSendSize++] = COM_ETX;

		//SP_xxxx_BOARD�ɑ΂��ăp�P�b�g���M�A�y�уf�[�^��M�҂�
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
								printf("(ch%d AD)OFFSET ���M(0x%X),��M(0x%X)�f�[�^�s��v\a\n", ii, dwAdOffset[ii], dwRecvOffset[ii]);
								berr = FALSE;	//���M�f�[�^�Ǝ�M�f�[�^���s��v
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
								printf("(ch%d AD)GAIN ���M(0x%X),��M(0x%X)�f�[�^�s��v\a\n", ii, dwAdGain[ii], dwRecvGain[ii]);
								berr = FALSE;	//���M�f�[�^�Ǝ�M�f�[�^���s��v
							}
						}
						if( dwReadGain != NULL ){
							dwReadGain[ii] = dwRecvGain[ii] & 0x0fff;
						}
					}
				}
			}
			else{
				printf("��M�f�[�^�`�F�b�N�T���G���[(Hit any key)\a\n");
				KeyInputWait();
				berr = FALSE;
			}
		}
#ifndef	_RS_DEBUG_MODE
		else{
			printf("��M�f�[�^�T�C�Y(=%d)�G���[(Hit any key)\a\n", nRecvSize);
			KeyInputWait();
			berr = FALSE;
		}
#endif
	}
	return berr;
}

////////////////////////////////////////
// �A�i���O�o�͕␳�l���X�V����
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

		//�d����
		sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
		nSendSize +=3;

		//DA�e�X�g���[�h
		m_cSendPacket[nSendSize++] = SP_RS_MODE_DAADJ;

		//�o�̓f�[�^�L���E�����t���O
		for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
			if( ii >= SP_XXXX_DACH_MAX ){
				m_cSendPacket[nSendSize++] = '0';		//�X�V�Ȃ�
			}
			else{
				if( dwDaOffset[ii] == 0xffff || dwDaGain[ii] == 0xffff ){
					m_cSendPacket[nSendSize++] = '0';		//�X�V�Ȃ�
				}
				else{
					m_cSendPacket[nSendSize++] = '1';		//�X�V����
					dwUpdateFlg[ii] = 0x8000;	//�X�V����
				}
			}
		}

		//�o�̓f�[�^
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

		//�`�F�b�N�T���v�Z
		ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
		sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
		nSendSize+=2;

		m_cSendPacket[nSendSize++] = COM_ETX;

		//SP_xxxx_BOARD�ɑ΂��ăp�P�b�g���M�A�y�уf�[�^��M�҂�
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
								printf("(ch%d DA)OFFSET ���M(0x%X),��M(0x%X)�f�[�^�s��v\a\n", ii, dwDaOffset[ii], dwRecvOffset[ii]);
								berr = FALSE;	//���M�f�[�^�Ǝ�M�f�[�^���s��v
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
								printf("(ch%d DA)GAIN ���M(0x%X),��M(0x%X)�f�[�^�s��v\a\n", ii, dwDaGain[ii], dwRecvGain[ii]);
								berr = FALSE;	//���M�f�[�^�Ǝ�M�f�[�^���s��v
							}
						}
						if( dwReadGain != NULL ){
							dwReadGain[ii] = dwRecvGain[ii] & 0x0fff;
						}
					}
				}
			}
			else{
				printf("��M�f�[�^�`�F�b�N�T���G���[(Hit any key)\a\n");
				KeyInputWait();
				berr = FALSE;
			}
		}
#ifndef	_RS_DEBUG_MODE
		else{
			printf("��M�f�[�^�T�C�Y(=%d)�G���[(Hit any key)\a\n", nRecvSize);
			KeyInputWait();
			berr = FALSE;
		}
#endif
	}
	return berr;
}


////////////////////////////////////////
// �t�@�[���E�F�A�̏������[�h��W�����[�h�ɖ߂�
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

	//�d����
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//�e�X�g���[�h
	m_cSendPacket[nSendSize++] = SP_RS_MODE_NORMAL;

	//�o�̓f�[�^�L���E�����t���O
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		m_cSendPacket[nSendSize++] = '0';	//�S�Ė���
	}

	//�o�̓f�[�^
	for(ii=0; ii<(SP_XXXX_ADCH_MAX*4); ii++){
		m_cSendPacket[nSendSize++] = '0';
	}

	//�`�F�b�N�T���v�Z
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARD�ɑ΂��ăp�P�b�g���M�A�y�уf�[�^��M�҂�
	nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
	if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD  ){
		ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
		ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
		if( ucCheckSumCalc == ucCheckSumRecv ){
			bret = TRUE;
		}
		else{
			printf("��M�f�[�^�`�F�b�N�T���G���[(Hit any key)\a\n");
			KeyInputWait();
		}
	}
	return bret;
}

////////////////////////////////////////
// �A�i���O���o�͕␳�l��EEPROM�ۑ����s
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

	//�d����
	sprintf((char*)&m_cSendPacket[nSendSize], "%03d", SP_FIXED_SEND_SIZE);
	nSendSize +=3;

	//�e�X�g���[�h
	m_cSendPacket[nSendSize++] = SP_RS_MODE_EEPSAVE;

	//�o�̓f�[�^�L���E�����t���O
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		if( ii >= nChMax ){
			m_cSendPacket[nSendSize++] = '0';		//�X�V�Ȃ�
		}
		else{
			m_cSendPacket[nSendSize++] = '1';		//�X�V����
		}
	}

	//�o�̓f�[�^
	for(ii=0; ii<SP_XXXX_ADCH_MAX; ii++){
		sprintf((char*)&m_cSendPacket[nSendSize], "%04X", 0x8000);
		nSendSize+=4;
	}

	//�`�F�b�N�T���v�Z
	ucCheckSumCalc = RS_SpCheckSum(m_cSendPacket, nSendSize);
	sprintf((char*)&m_cSendPacket[nSendSize], "%02X", ucCheckSumCalc);
	nSendSize+=2;

	m_cSendPacket[nSendSize++] = COM_ETX;

	//SP_xxxx_BOARD�ɑ΂��ăp�P�b�g���M�A�y�уf�[�^��M�҂�
	nRecvSize = RS_SpSendRecv(m_cSendPacket, nSendSize, m_cRecvPacket, SEND_RECV_PAKET_SIZE);
	if( nRecvSize == SP_FIXED_RECV_SIZE || nRecvSize == SP_FIXED_RECV_SIZE_OLD ){
		ucCheckSumCalc = RS_SpCheckSum(m_cRecvPacket, nRecvSize-3);
		ucCheckSumRecv = (unsigned char)strtol((char*)&m_cRecvPacket[nRecvSize-3], &endptr, 16);
		if( ucCheckSumCalc != ucCheckSumRecv ){
			printf("��M�f�[�^�`�F�b�N�T���G���[(Hit any key)\a\n");
			KeyInputWait();
			berr = FALSE;
		}
	}
#ifndef	_RS_DEBUG_MODE
	else{
		printf("��M�f�[�^�T�C�Y(=%d)�G���[(Hit any key)\a\n", nRecvSize);
		KeyInputWait();
		berr = FALSE;
	}
#endif
	return berr;
}

////////////////////////////////////////
// RS-232C����M�f�[�^�̃`�F�b�N�T�����v�Z����
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
// ������Ƃ�RS-232C�ʐM�|�[�g���I�[�v������
BOOL RS_SpComOpen()
{
	DCB dcb;					//COM�߰�DCB�\����
	COMMTIMEOUTS timeouts;		//COMM�߰���ѱ�č\����
	COMMPROP cp;				//COM������č\����
	DWORD dwerr;
	
	if( m_hSpComHandle != INVALID_HANDLE_VALUE ){
		CloseHandle(m_hSpComHandle);
		m_hSpComHandle = INVALID_HANDLE_VALUE;
	}
	
	m_hSpComHandle = CreateFile(m_cSpxxComPort, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if( m_hSpComHandle == INVALID_HANDLE_VALUE ){
		dwerr = GetLastError();
		printf("%s �|�[�g�I�[�v���G���[(err=0x%X)\a\n", m_cSpxxComPort, dwerr );
		return FALSE;			//COMM�߰ĵ���ݴװ
	}

	cp.wPacketLength = sizeof(COMMPROP);					// COMMPROP�̻���
	GetCommProperties (m_hSpComHandle, &cp);				// COMM�߰Ă������è
	GetCommState( m_hSpComHandle, &dcb );				// COM�߰ď����擾
	dcb.BaudRate = 38400;								// �ʐM���x
	dcb.ByteSize = 8;									// �f�[�^��
	dcb.StopBits = ONESTOPBIT;							// �X�g�b�v�r�b�g
	dcb.fParity = 0;									// �p���e�B�`�F�b�N 0:����
	dcb.Parity = NOPARITY;								// �p���e�B�Ȃ�
	SetCommState( m_hSpComHandle, &dcb );				// COM�|�[�g�����ݒ�

	if (!(cp.dwProvCapabilities & PCF_TOTALTIMEOUTS)){
		//���̃|�[�g��Total�^�C���A�E�g���T�|�[�g���Ă��Ȃ�
		CloseHandle(m_hSpComHandle);
		m_hSpComHandle = INVALID_HANDLE_VALUE;
		printf("%s �|�[�g�I�[�v���G���[???\a\n", m_cSpxxComPort);
		return FALSE;
	}

	memset(&timeouts, 0, sizeof(timeouts));
	GetCommTimeouts(m_hSpComHandle, &timeouts);	// ��ѱ�Đݒ�擾
	timeouts.ReadIntervalTimeout = 20;			//2�����Ԃ̍ő�҂�����
	timeouts.WriteTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 20;	//Write�^�C���A�E�g���Ԃ̊|���Z�W��;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.ReadTotalTimeoutMultiplier = 20;	//Read�^�C���A�E�g���Ԃ̊|���Z�W��;
	SetCommTimeouts(m_hSpComHandle, &timeouts);	// ��ѱ�Đݒ�

	return TRUE;
}

//////////////////////////////////////////////
//SP_xxxx_BOARD RS-232C�f�[�^����M
int RS_SpSendRecv(unsigned char *cSendPacket, int nSendSize, unsigned char *cRecvPacket, int nRecvSizeMax)
{
	DWORD	nActualWrite;	//���ۂɏ������񂾃o�C�g��
	DWORD	nBytesRead;
	DWORD	dwError;
	COMSTAT	cs;
	BOOL	etx_rcvd=FALSE;
	int		ii;
	int		nRetRecvSize=-1;

	if( m_hSpComHandle == INVALID_HANDLE_VALUE ){
		return nRetRecvSize;	//COM�|�[�g�I�[�v���G���[
	}
	PurgeComm( m_hSpComHandle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );

	nActualWrite = 0;
	for( ii=0; ii<3; ii++ ){
		//SP_xxxx_BOARD�Ƀp�P�b�g���M
		WriteFile(m_hSpComHandle, cSendPacket, nSendSize, &nActualWrite, NULL);
		if( (DWORD)nSendSize == nActualWrite ){
			break;
		}
	}

	//SP_xxxx_BOARD����̉����҂�
	if( (DWORD)nSendSize == nActualWrite ){
#ifndef	_RS_DEBUG_MODE
		//STX��M�҂�
		for( ii=0; ii<3; ii++ ){
			Sleep_Cnt(50);
			if( ReadFile(m_hSpComHandle, &cRecvPacket[0], 1, &nBytesRead, NULL) == 0 ){
				ClearCommError(m_hSpComHandle, &dwError, &cs);	// ��M�G���[�̌㏈��
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

		//ETX��M�҂�
		if( nRetRecvSize == 1 ){
			for( ii=1; ii<nRecvSizeMax; ii++ ){
				if( ReadFile(m_hSpComHandle, &cRecvPacket[ii], 1, &nBytesRead, NULL) == 0 ){
					ClearCommError(m_hSpComHandle, &dwError, &cs);	// ��M�G���[�̌㏈��
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
// SN-4016-SRCM, SN-4016-STCM �r�b�g�ԍ���MIL20�s���̃s���ԍ��ϊ�
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
	if( wAdr > 12 ){	//12:�����i�̍ŏIAdr
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
	if( wAdr >= SP_XXXX_SW1_START_ADR && wAdr <= 12 ){	//12:�����i�̍ŏIAdr
		sprintf(m_cSpInternalSignalName, "(%s%d-%x)", strInOrOut, (wAdr-SP_XXXX_SW1_START_ADR+1), wBitNum);
	}
	return m_cSpInternalSignalName;
}

////////////////////////////////////////////////////
//�}���`���[�^����d���l��RS232C�o�R�œǂݎ��
BOOL GetVoltageVal(HANDLE hhandle, double *val)
{
	double	dwk1 = -1;
	BOOL	retGetMeasure_HP34401A;
	BOOL	berr=TRUE;

	//�}���`���[�^����d���l���擾����
	retGetMeasure_HP34401A = GetMeasure_HP34401A(
		hhandle			//�ʐM���\�[�X�n���h��
		,&dwk1			//�}���`���[�^�v���l
	);
	if( ! retGetMeasure_HP34401A ){
		LogPrintf("MultiMeter 34401A Data Read Error!!\n");
		berr = FALSE;
	}

	if( berr == TRUE ){
		//*val = dwk1 * 1000.0;	//�擾�l��V�l�ɕϊ�����
		*val = dwk1;
	}
	return berr;
}

////////////////////////////////////////////////////
// DA�o�̓}���`���[�^�l�̍ŏ��A�ő�A�Z���^�[�����̕��ς��擾����
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
//�A�i���O���͂̕␳�l�i�I�t�Z�b�g�E�Q�C���j���Z�o����
//#define	HARDWARE_FACTOR		(38496)		//�n�[�h��̗��_�I�ȌW���i0x9660�j
//#define	HARDWARE_FACTOR_4MA	(8205)		//�n�[�h��̗��_�I��4mA���͒l�i0x200D�j
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

	//�Q�C��
	*dwAdGain = 0;
	if( AdAveVal20mA > AdAveVal4mA ){
		d_wk = (double)(AdAveVal20mA - AdAveVal4mA);			//4mA �� 20mA���̐��f�[�^�ω���

		d_wk = (double)(16384 * 65536) / d_wk;		//���f�[�^�ω�����0�`4000H���ɕϊ�����W��
		
		d_gain_adj = (double)((double)(d_wk * 65536.0) / d_gain_factor);	//�Q�C���␳�l 

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

	//�I�t�Z�b�g
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
// AD���͒l�̍ŏ��A�ő�A�Z���^�[�����̕��ς��擾����
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
//	���O�t�@�C���L�^�Ɖ�ʕ\��
//
////////////////////////////////////////////////////////////////////
void LogPrintfCsv(const char *fname, const char * Format, ...)
{
	char	cLogName[_MAX_PATH];
	FILE* fp;
	va_list vl;

	va_start(vl, Format);

	// ��ʕ\��
	vprintf(Format, vl);

	// �t�@�C���ۑ�
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

	// ��ʕ\��
	vprintf(Format, vl);

	// �t�@�C���ۑ�
	fp = fopen("SPxxxxLog.txt", "a");
	if(fp){
		vfprintf(fp, Format, vl);
		fclose(fp);
	}

	va_end(vl);
}


//���l�i������j���͑҂�
void KeyStrInputWait(char *strbuf)
{
	KeyBufClear();		//�L�[�o�b�t�@����ɂ���
	printf("\a");
	gets(strbuf);
}

//�L�[���͑҂�
int KeyInputWait()
{
	int nKeyIn = 0;

	KeyBufClear();		//�L�[�o�b�t�@����ɂ���
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
			continue;		//���������͂��ꂽ�ꍇ�͍ē��͑҂��Ƃ���
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

//�L�[���̓o�b�t�@�N���A
void KeyBufClear()
{
	while(ForeverLoop){
		//�L�[�o�b�t�@����ɂ���
		if( _kbhit() ){
			_getch();
		}
		else{
			break;
		}
	}
}
//�����L�[�̉����҂�
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

//���b�g�t�H���_��������ΐ�������
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
		{	//�p�X�����݂��Ȃ��Ƃ��́A�t�H���_�𐶐�����
			if( CreateDirectory(cLogFolder, NULL) == 0 )
			{	//�f�B���N�g���̐����Ɏ��s�����ꍇ�̓��O�o�͂��Ȃ�
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
		{	//�p�X�����݂��Ȃ��Ƃ��́A�t�H���_�𐶐�����
			if( CreateDirectory(m_cLogFolderName, NULL) == 0 )
			{	//�f�B���N�g���̐����Ɏ��s�����ꍇ�̓��O�o�͂��Ȃ�
				memset(m_cLogFolderName, 0, sizeof(m_cLogFolderName));
			}
		}
		
		//ini����RS-232C�ʐM�|�[�g�ԍ����擾����
		(void)_stprintf(szIni, "%s%sParam.ini", szDrive, szDir);
	}
	GetPrivateProfileString("PARAM", "ComPort", "COM1", m_cComPort, sizeof(m_cComPort), szIni);
	GetPrivateProfileString("PARAM", "SPxxComPort", "COM2", m_cSpxxComPort, sizeof(m_cSpxxComPort), szIni);
	m_nIniBoardID = (int)GetPrivateProfileInt("PARAM", "SP_BOARD_TYPE_DEF", -1, szIni);
	m_nIniSpFrontBoardType = (int)GetPrivateProfileInt("PARAM", "SP_FRONT_IF_BOARD_TYPE", 1, szIni);	//Default��Front2
	m_nIniSpUpperBoardType = (int)GetPrivateProfileInt("PARAM", "SP_UPPER_IF_BOARD_TYPE", 1, szIni);	//Default��Upper2
	m_nIniSpMotherBoardType = (int)GetPrivateProfileInt("PARAM", "SP_MOTHER_BOARD_TYPE", 0, szIni);
	//m_nIniSpHlsTestDswAllOff = (int)GetPrivateProfileInt("PARAM", "SP_HLSTEST_DSW_ALL_OFF", 1, szIni);

	GetPrivateProfileString("PARAM", "AIO_UNIT_DEVICE_NAME", "AIO000", m_cIniAioUnitDeviceName, sizeof(m_cIniAioUnitDeviceName), szIni);

	//CONTEC AIO���j�b�g�p�̃p�����[�^����͂���(�d���o��)
	m_nIniAO_0_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AO_0_0V", 0, szIni);	
	m_nIniAO_2_5V = (int)GetPrivateProfileInt("AIO_UNIT", "AO_2_5V", 0, szIni);	
	m_nIniAO_5_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AO_5_0V", 0, szIni);	
	m_nIniAO_10_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AO_10_0V", 0, szIni);	
	m_nIniAO_0_OVR_L = (int)GetPrivateProfileInt("AIO_UNIT", "AO_0_OVR_L", 0, szIni);	
	m_nIniAO_5_OVR_H = (int)GetPrivateProfileInt("AIO_UNIT", "AO_5_OVR_H", 0, szIni);	
	m_nIniAO_10_OVR_H = (int)GetPrivateProfileInt("AIO_UNIT", "AO_10_OVR_H", 0, szIni);	

	//CONTEC AIO���j�b�g�p�̃p�����[�^����͂���(�d������)
	m_nIniAI_0_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AI_0_0V", 0, szIni);	
	m_nIniAI_10_0V = (int)GetPrivateProfileInt("AIO_UNIT", "AI_10_0V", 0, szIni);	
	//0-10V���͒�����Ԃ̌X�����v�Z����
	m_dContecAiAdjGainV = 10.0 / (double)(m_nIniAI_10_0V-m_nIniAI_0_0V);
	//0-10V���͒�����Ԃ̐ؕЂ��v�Z����
	m_dContecAiAdjOffsetV = 10.0 - (double)m_nIniAI_10_0V * m_dContecAiAdjGainV;

	//CONTEC AIO���j�b�g�p�̃p�����[�^����͂���(�d���o��)
	m_nIniAO_4mA = (int)GetPrivateProfileInt("AIO_UNIT", "AO_4mA", 0, szIni);	
	m_nIniAO_12mA = (int)GetPrivateProfileInt("AIO_UNIT", "AO_12mA", 0, szIni);	
	m_nIniAO_20mA = (int)GetPrivateProfileInt("AIO_UNIT", "AO_20mA", 0, szIni);	
	m_nIniAO_4mA_OVR_L = (int)GetPrivateProfileInt("AIO_UNIT", "AO_4mA_OVR_L", 0, szIni);	
	m_nIniAO_20mA_OVR_H = (int)GetPrivateProfileInt("AIO_UNIT", "AO_20mA_OVR_H", 0, szIni);	

	//CONTEC AIO���j�b�g�p�̃p�����[�^����͂���(�d������)
	m_nIniAI_4mA = (int)GetPrivateProfileInt("AIO_UNIT", "AI_4mA", 0, szIni);	
	m_nIniAI_20mA = (int)GetPrivateProfileInt("AIO_UNIT", "AI_20mA", 0, szIni);	
	//4-20mA���͒�����Ԃ̌X�����v�Z����
	m_dContecAiAdjGainI = (20.0 - 4.0) / (double)(m_nIniAI_20mA-m_nIniAI_4mA);
	//4-20mA���͒�����Ԃ̐ؕЂ��v�Z����
	m_dContecAiAdjOffsetI = 20.0 - (double)m_nIniAI_20mA * m_dContecAiAdjGainI;
}

////////////////////////////////////////////////////////////////////
//
//          �v���O�����ُ�I������
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
        CloseVxD();                     // OpenVxD()�����������ꍇ�A
                                        // �I���O��CloseVxD��K�����s
    }
    FreeLibrary(hDll);                  // DLL���J������

	printf("\nInput any key....");
	HitAnyKeyWait();
    exit(-1);                           // �v���O�����I��
}

////////////////////////////////////////////////////////////////////
//
//          �h���C�oDLL�̃��[�h�A�G���g���[�擾
//
////////////////////////////////////////////////////////////////////
BOOLEAN LoadSnDll(const char * pcFileName)
{
	char pcFile[32];

	if(pcFileName[0] == '\0'){
		return FALSE;
	}

	sprintf(pcFile, "%s.DLL", pcFileName);

	// DLL�����[�h
	hDll = LoadLibrary(pcFile);
	if (hDll == NULL) {
		LogPrintf("Load %s.DLL failed %08X\n", pcFileName, GetLastError());
		return(FALSE);
	}

	printf("Load %s.DLL Instance Handle = %08X\n", pcFileName, hDll);

    // Entry Point�̎擾
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

    // Entry Point�̊m�F
    if ((OpenVxD == NULL) || (CloseVxD == NULL) || (GETversion == NULL) || (BOARDinit == NULL) ||
        (TERMstart == NULL) || (TERMstop == NULL) || (TERMchk == NULL) || (BOARDchk == NULL) ||
        (UNITcntr == NULL) || (DATAin == NULL) || (DATAout == NULL) || (DATAoutW == NULL) ||
        (ADin == NULL) || (DAout == NULL) || (CNTin == NULL) || (CNT0 == NULL) ||
        (TERMread == NULL) || (TERMwrite == NULL) || (RETRYread == NULL) || (RETRYwrite == NULL) ||
        (WIREread == NULL) || (WIREwrite == NULL) || (EEPROMwrite == NULL) || (WIREswitch == NULL) ||
        (WIREon == NULL) || (WIREoff == NULL) || (WIREchk == NULL) || (BOARDlist == NULL) ||
        (TERMsetup == NULL) || (MltAdIn == NULL) || (GetMltAdData == NULL)) {

        // Entry Point�̎擾�Ɏ��s�����ꍇ�G���[�I��
		LogPrintf("Failed to Get %s.DLL Entry Point!!\n", pcFileName);
        FreeLibrary(hDll);
        return(FALSE);
    }
    return(TRUE);
}

////////////////////////////////////////////////////////////////////
//
//	�v���O���X�o�[����ʕ\��
//
////////////////////////////////////////////////////////////////////
void WaitProgressDot(
	DWORD	Interval,	// �~���b�Ԋu��.��\��
	DWORD	DotMax)		// . ��\�������
{
	DWORD lp;

	for(lp = 0; lp < DotMax; lp++) {
		printf(".");
		Sleep_Cnt(Interval);
	}
}

////////////////////////////////////////////////////////////////////
//	CONTEC AIO-160802AY-USB�֘A�̏���
////////////////////////////////////////////////////////////////////

///////////////////////////////////
//AIO���j�b�g������
BOOL	AioUnitOpen()
{
	long	lAioRet;
	//float	fAiVal;
	//short	AiMaxChannels;
	//short	AoMaxChannels;

	//������
	if( m_shAioUnitId != -1 ){
		AioExit(m_shAioUnitId);
	}

	lAioRet = AioInit(m_cIniAioUnitDeviceName, &m_shAioUnitId);
    if(lAioRet != 0){
		AioGetErrorString(lAioRet , m_cAioStringWork);
		printf("AioInit(%ld):%s\n", lAioRet, m_cAioStringWork);
		printf("�����L�[�������Ă�������\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}
	AioResetDevice(m_shAioUnitId);	//�f�o�C�X�ݒ��������Ԃɖ߂�

	//���̓����W�̐ݒ�
	lAioRet = AioSetAiRangeAll(m_shAioUnitId, PM10);	//AIO-160802AY-USB�́}10V�Œ�
	if(lAioRet != 0){
		AioGetErrorString(lAioRet, m_cAioStringWork);
		printf("AioSetAiRangeAll(%ld): %s\n", lAioRet, m_cAioStringWork);
		AioExit(m_shAioUnitId);
		printf("�����L�[�������Ă�������\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}
	
	//�o�̓����W�̐ݒ�
	lAioRet = AioSetAoRangeAll(m_shAioUnitId, PM10);	//AIO-160802AY-USB�́}10V�Œ�
	if(lAioRet != 0){
		AioGetErrorString(lAioRet, m_cAioStringWork);
		printf("AioSetAoRangeAll(%ld): %s\n", lAioRet, m_cAioStringWork);
		AioExit(m_shAioUnitId);
		printf("�����L�[�������Ă�������\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}

	//�A�i���O���̓`���l���̐ݒ�(�g�p����ch����8�`���l��(MAX=8))
	//lAioRet = AioGetAiMaxChannels( m_shAioUnitId , &AiMaxChannels );
	lAioRet = AioSetAiChannels(m_shAioUnitId , 8);
	if(lAioRet != 0){
		AioGetErrorString(lAioRet, m_cAioStringWork);
		printf("AioSetAiChannels(%ld): %s\n", lAioRet, m_cAioStringWork);
		AioExit(m_shAioUnitId);
		printf("�����L�[�������Ă�������\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}

	//�A�i���O�o�̓`���l���̐ݒ�(�g�p����ch����1�`���l��(MAX=2))
	//lAioRet = AioGetAoMaxChannels( m_shAioUnitId , &AoMaxChannels );
	lAioRet = AioSetAoChannels(m_shAioUnitId , 1);
	if(lAioRet != 0){
		AioGetErrorString(lAioRet, m_cAioStringWork);
		printf("AioSetAoChannels(%ld): %s\n", lAioRet, m_cAioStringWork);
		AioExit(m_shAioUnitId);
		printf("�����L�[�������Ă�������\a\n");
		HitAnyKeyWait();
    	return FALSE;
	}
	else{
		//�����o�͂�0V�Ƃ���
		AioSingleAo( m_shAioUnitId , 0 , m_nIniAO_0_0V );
	}
	//�܂�Ԃ�Test
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
//AIO���j�b�g�I��
void	AioUnitClose()
{
	if( m_shAioUnitId != -1 ){
		AioExit(m_shAioUnitId);
		m_shAioUnitId = -1;
	}
}

///////////////////////////////////
//AIO���j�b�g�ɑ΂��Ďw�肳�ꂽ�d���l���o�͂���
void	AioUnitWriteAOVal(short sAOch, long lAOval)
{
	BOOL	bOpenOK = TRUE;
	long	lAioRet;
	if( m_shAioUnitId == -1 ){
		bOpenOK = AioUnitOpen();
	}

	if( bOpenOK == TRUE ){
		//�d���o�͂��s��
		lAioRet = AioSingleAo( m_shAioUnitId, sAOch, lAOval );
		if(lAioRet != 0){
			AioGetErrorString(lAioRet, m_cAioStringWork);
			printf("AioSingleAo(%ld): %s\n", lAioRet, m_cAioStringWork);
			printf("�����L�[�������Ă�������\a\n");
			HitAnyKeyWait();
		}
	}
}

///////////////////////////////////
//AIO���j�b�g����d���l��ǂݏo��
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
		//�d���l��ǂݏo��
		lAioRet = AioSingleAi( m_shAioUnitId, sAIch, &lAIval );
		if(lAioRet != 0){
			AioGetErrorString(lAioRet, m_cAioStringWork);
			printf("AioSingleAo(%ld): %s\n", lAioRet, m_cAioStringWork);
			printf("�����L�[�������Ă�������\a\n");
			HitAnyKeyWait();
		}
		else{
			//�ǂݏo�����l(16bit)��d���l�ɕϊ�����(�������)
			*dAIval = (double)((double)lAIval * m_dContecAiAdjGainV) + m_dContecAiAdjOffsetV;
		}
	}
}
