/*
*********************************************************************************************************
*
*	Ä£¿éÃû³Æ : SD¿¨FatÎÄ¼þÏµÍ³ºÍSD¿¨Ä£ÄâÑÝÊ¾Ä£¿é¡£
*	ÎÄ¼þÃû³Æ : demo_sd_fatfs.c
*	°æ    ±¾ : V1.0
*	Ëµ    Ã÷ : ¸ÃÀý³ÌÒÆÖ²FatFSÎÄ¼þÏµÍ³£¨°æ±¾ R0.12c£©£¬ÑÝÊ¾ÈçºÎ´´½¨ÎÄ¼þ¡¢¶ÁÈ¡ÎÄ¼þ¡¢´´½¨Ä¿Â¼ºÍÉ¾³ýÎÄ¼þ
*			   ²¢²âÊÔÁËÎÄ¼þ¶ÁÐ´ËÙ¶È.Ö§³ÖÒÔÏÂ6¸ö¹¦ÄÜ£¬ÓÃ»§Í¨¹ýµçÄÔ¶Ë´®¿ÚÈí¼þ·¢ËÍÊý×Ö¸ø¿ª·¢°å¼´¿É:
*              1 - ÏÔÊ¾¸ùÄ¿Â¼ÏÂµÄÎÄ¼þÁÐ±í
*              2 - ´´½¨Ò»¸öÐÂÎÄ¼þarmfly.txt
*              3 - ¶Áarmfly.txtÎÄ¼þµÄÄÚÈÝ
*              4 - ´´½¨Ä¿Â¼
*              5 - É¾³ýÎÄ¼þºÍÄ¿Â¼
*              6 - ¶ÁÐ´ÎÄ¼þËÙ¶È²âÊÔ
*              a - ´ò¿ªSD¿¨Ä£ÄâUÅÌ
*              b - ¹Ø±ÕSD¿¨Ä£ÄâUÅÌ
*
*              ÁíÍâ×¢Òâ£¬SD¿¨Ä£ÄâUÅÌÊÇÓÃµÄMicroUSB½Ó¿Ú¡£
*	ÐÞ¸Ä¼ÇÂ¼ :
*		°æ±¾ºÅ   ÈÕÆÚ         ×÷Õß        ËµÃ÷
*		V1.0    2018-12-12   Eric2013    ÕýÊ½·¢²¼
*
*	Copyright (C), 2018-2030, °²¸»À³µç×Ó www.armfly.com
*
*********************************************************************************************************
*/
#include "bsp.h"
#include "ff.h"				/* FatFSÎÄ¼þÏµÍ³Ä£¿é*/
#include "ff_gen_drv.h"
#include "sd_diskio_dma.h"
//#include "usbd_usr.h"


/* ÓÃÓÚ²âÊÔ¶ÁÐ´ËÙ¶È */
#define TEST_FILE_LEN			(2*1024*1024)	/* ÓÃÓÚ²âÊÔµÄÎÄ¼þ³¤¶È */
#define BUF_SIZE				(4*1024)		/* Ã¿´Î¶ÁÐ´SD¿¨µÄ×î´óÊý¾Ý³¤¶È */

/* ½öÔÊÐí±¾ÎÄ¼þÄÚµ÷ÓÃµÄº¯ÊýÉùÃ÷ */
static void DispMenu(void);
static void ViewRootDir(void);
static void CreateNewFile(void);
static void ReadFileData(void);
static void CreateDir(void);
static void DeleteDirFile(void);
static void WriteFileTest(void);

/*
    1¡¢V7¿ª·¢°åµÄSD¿¨½Ó¿ÚÊÇÓÃµÄSDMMC1£¬¶øÕâ¸ö½Ó¿Ú½öÖ§³ÖAXI SRAMÇø·ÃÎÊ£¬ÆäËüSRAMºÍTCP¾ù²»Ö§³Ö¡£
    2¡¢AXI SRAMµÄÖ÷ÆµÊÇ200MHz£¬ÐèÒªCacheÅäºÏÌáËÙ£¬ËùÒÔ´ËÀý×ÓÖ÷RAMÖ±½ÓÊ¹ÓÃÐÔÄÜ×î¸ßµÄDTCM¡£½öDMA
       ²Ù×÷SDIOµÄµØ·½Ê¹ÓÃAXI SRAM£¬ÏÂÃæÕâ¼¸¸ö±äÁ¿ÊÇ¶¼ÐèÒªÊ¹ÓÃAXI SRAM¿Õ¼äµÄ¡£
    3¡¢ÏêÇéÔÚ´ËÌù½øÐÐÁËËµÃ÷£ºhttp://forum.armfly.com/forum.php?mod=viewthread&tid=91531
*/
#if defined ( __ICCARM__ )
  #pragma location = 0x24000000
  FATFS fs;
  #pragma location = 0x24001000
  FIL file;
  #pragma location = 0x24002000
  char FsReadBuf[1024];
  #pragma location = 0x24003000
  char FsWriteBuf[1024] = {"FatFS Write Demo \r\n www.armfly.com \r\n"};
  #pragma location = 0x24004000
  uint8_t g_TestBuf[BUF_SIZE];
#elif defined( __CC_ARM ) || defined(__GNUC__)
__attribute__((section (".RAM_D1")))	FATFS fs;
__attribute__((section (".RAM_D1")))	FIL file;
ALIGN_32BYTES(__attribute__((section (".RAM_D1"))) char FsReadBuf[1024]);
ALIGN_32BYTES(__attribute__((section (".RAM_D1"))) char FsWriteBuf[1024]) = {"FatFS Write Demo \r\n www.armfly.com \r\n"};
ALIGN_32BYTES(__attribute__((section (".RAM_D1"))) uint8_t g_TestBuf[BUF_SIZE]);
#endif

DIR DirInf;
FILINFO FileInf;
char DiskPath[4]; /* SD¿¨Âß¼­Çý¶¯Â·¾¶£¬±ÈÅÌ·û0£¬¾ÍÊÇ"0:/" */

/* FatFs APIµÄ·µ»ØÖµ */
static const char * FR_Table[]= 
{
	"FR_OK£º³É¹¦",				                             /* (0) Succeeded */
	"FR_DISK_ERR£ºµ×²ãÓ²¼þ´íÎó",			                 /* (1) A hard error occurred in the low level disk I/O layer */
	"FR_INT_ERR£º¶ÏÑÔÊ§°Ü",				                     /* (2) Assertion failed */
	"FR_NOT_READY£ºÎïÀíÇý¶¯Ã»ÓÐ¹¤×÷",			             /* (3) The physical drive cannot work */
	"FR_NO_FILE£ºÎÄ¼þ²»´æÔÚ",				                 /* (4) Could not find the file */
	"FR_NO_PATH£ºÂ·¾¶²»´æÔÚ",				                 /* (5) Could not find the path */
	"FR_INVALID_NAME£ºÎÞÐ§ÎÄ¼þÃû",		                     /* (6) The path name format is invalid */
	"FR_DENIED£ºÓÉÓÚ½ûÖ¹·ÃÎÊ»òÕßÄ¿Â¼ÒÑÂú·ÃÎÊ±»¾Ü¾ø",         /* (7) Access denied due to prohibited access or directory full */
	"FR_EXIST£ºÎÄ¼þÒÑ¾­´æÔÚ",			                     /* (8) Access denied due to prohibited access */
	"FR_INVALID_OBJECT£ºÎÄ¼þ»òÕßÄ¿Â¼¶ÔÏóÎÞÐ§",		         /* (9) The file/directory object is invalid */
	"FR_WRITE_PROTECTED£ºÎïÀíÇý¶¯±»Ð´±£»¤",		             /* (10) The physical drive is write protected */
	"FR_INVALID_DRIVE£ºÂß¼­Çý¶¯ºÅÎÞÐ§",		                 /* (11) The logical drive number is invalid */
	"FR_NOT_ENABLED£º¾íÖÐÎÞ¹¤×÷Çø",			                 /* (12) The volume has no work area */
	"FR_NO_FILESYSTEM£ºÃ»ÓÐÓÐÐ§µÄFAT¾í",		             /* (13) There is no valid FAT volume */
	"FR_MKFS_ABORTED£ºÓÉÓÚ²ÎÊý´íÎóf_mkfs()±»ÖÕÖ¹",	         /* (14) The f_mkfs() aborted due to any parameter error */
	"FR_TIMEOUT£ºÔÚ¹æ¶¨µÄÊ±¼äÄÚÎÞ·¨»ñµÃ·ÃÎÊ¾íµÄÐí¿É",		 /* (15) Could not get a grant to access the volume within defined period */
	"FR_LOCKED£ºÓÉÓÚÎÄ¼þ¹²Ïí²ßÂÔ²Ù×÷±»¾Ü¾ø",				 /* (16) The operation is rejected according to the file sharing policy */
	"FR_NOT_ENOUGH_CORE£ºÎÞ·¨·ÖÅä³¤ÎÄ¼þÃû¹¤×÷Çø",		     /* (17) LFN working buffer could not be allocated */
	"FR_TOO_MANY_OPEN_FILES£ºµ±Ç°´ò¿ªµÄÎÄ¼þÊý´óÓÚ_FS_SHARE", /* (18) Number of open files > _FS_SHARE */
	"FR_INVALID_PARAMETER£º²ÎÊýÎÞÐ§"	                     /* (19) Given parameter is invalid */
};

/*
*********************************************************************************************************
*	º¯ Êý Ãû: DemoFatFS
*	¹¦ÄÜËµÃ÷: FatFSÎÄ¼þÏµÍ³ÑÝÊ¾Ö÷³ÌÐò
*	ÐÎ    ²Î: ÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
void DemoFatFS(void)
{
	uint8_t cmd;

	/* ´òÓ¡ÃüÁîÁÐ±í£¬ÓÃ»§¿ÉÒÔÍ¨¹ý´®¿Ú²Ù×÷Ö¸Áî */
	//DispMenu();
	
	/* ×¢²áSD¿¨Çý¶¯ */
	FATFS_LinkDriver(&SD_Driver, DiskPath);
	CreateNewFile();
	
//	bsp_StartAutoTimer(0, 500);	/* Æô¶¯1¸ö500msµÄ×Ô¶¯ÖØ×°µÄ¶¨Ê±Æ÷ */
	
//	while (1)
//	{
//		
//		/* ÅÐ¶Ï¶¨Ê±Æ÷³¬Ê±Ê±¼ä */
//		if (bsp_CheckTimer(0))	
//		{            
//			/* Ã¿¸ô500ms ½øÀ´Ò»´Î */  
//			bsp_LedToggle(2);
//		}

//		if (comGetChar(COM1, &cmd))	/* ´Ó´®¿Ú¶ÁÈëÒ»¸ö×Ö·û(·Ç×èÈû·½Ê½) */
//		{
//			printf("\r\n");
//			switch (cmd)
//			{
//				case '1':
//					printf("¡¾1 - ViewRootDir¡¿\r\n");
//					ViewRootDir();		/* ÏÔÊ¾SD¿¨¸ùÄ¿Â¼ÏÂµÄÎÄ¼þÃû */
//					break;

//				case '2':
//					printf("¡¾2 - CreateNewFile¡¿\r\n");
//					CreateNewFile();	/* ´´½¨Ò»¸öÐÂÎÄ¼þ,Ð´ÈëÒ»¸ö×Ö·û´® */
//					break;

//				case '3':
//					printf("¡¾3 - ReadFileData¡¿\r\n");
//					ReadFileData();		/* ¶ÁÈ¡¸ùÄ¿Â¼ÏÂarmfly.txtµÄÄÚÈÝ */
//					break;

//				case '4':
//					printf("¡¾4 - CreateDir¡¿\r\n");
//					CreateDir();		/* ´´½¨Ä¿Â¼ */
//					break;

//				case '5':
//					printf("¡¾5 - DeleteDirFile¡¿\r\n");
//					DeleteDirFile();	/* É¾³ýÄ¿Â¼ºÍÎÄ¼þ */
//					break;

//				case '6':
//					printf("¡¾6 - TestSpeed¡¿\r\n");
//					WriteFileTest();	/* ËÙ¶È²âÊÔ */
//					break;
//				
//				case 'a':
//					printf("¡¾a - OpenUSB¡¿\r\n");
//					usbd_OpenMassStorage();
//					break;
//				
//				case 'b':
//					printf("¡¾b - CloseUSB¡¿\r\n");
//					usbd_CloseMassStorage();
//					break;
//				
//				default:
//					DispMenu();
//					break;
//			}
//		}
//	}
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: DispMenu
*	¹¦ÄÜËµÃ÷: ÏÔÊ¾²Ù×÷ÌáÊ¾²Ëµ¥
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
static void DispMenu(void)
{
	printf("\r\n------------------------------------------------\r\n");
	printf("ÇëÑ¡Ôñ²Ù×÷ÃüÁî£¬´ò¿ªSD¿¨Ä£ÄâUÅÌ²Ù×÷ÆÚ¼ä²»Ö§³ÖÔÙµ÷ÓÃÃüÁî1-6:\r\n");
	printf("1 - ÏÔÊ¾¸ùÄ¿Â¼ÏÂµÄÎÄ¼þÁÐ±í\r\n");
	printf("2 - ´´½¨Ò»¸öÐÂÎÄ¼þarmfly.txt\r\n");
	printf("3 - ¶Áarmfly.txtÎÄ¼þµÄÄÚÈÝ\r\n");
	printf("4 - ´´½¨Ä¿Â¼\r\n");
	printf("5 - É¾³ýÎÄ¼þºÍÄ¿Â¼\r\n");
	printf("6 - ¶ÁÐ´ÎÄ¼þËÙ¶È²âÊÔ\r\n");
	printf("a - ´ò¿ªSD¿¨Ä£ÄâUÅÌ\r\n");
	printf("b - ¹Ø±ÕSD¿¨Ä£ÄâUÅÌ\r\n");
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: ViewRootDir
*	¹¦ÄÜËµÃ÷: ÏÔÊ¾SD¿¨¸ùÄ¿Â¼ÏÂµÄÎÄ¼þÃû
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
extern SD_HandleTypeDef uSdHandle;
static void ViewRootDir(void)
{
	FRESULT result;
	uint32_t cnt = 0;
	FILINFO fno;
	
	
 	/* ¹ÒÔØÎÄ¼þÏµÍ³ */
	result = f_mount(&fs, DiskPath, 0);	/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("¹ÒÔØÎÄ¼þÏµÍ³Ê§°Ü (%s)\r\n", FR_Table[result]);
	}

	/* ´ò¿ª¸ùÎÄ¼þ¼Ð */
	result = f_opendir(&DirInf, DiskPath); /* Èç¹û²»´ø²ÎÊý£¬Ôò´Óµ±Ç°Ä¿Â¼¿ªÊ¼ */
	if (result != FR_OK)
	{
		printf("´ò¿ª¸ùÄ¿Â¼Ê§°Ü  (%s)\r\n", FR_Table[result]);
		return;
	}

	printf("ÊôÐÔ        |  ÎÄ¼þ´óÐ¡ | ¶ÌÎÄ¼þÃû | ³¤ÎÄ¼þÃû\r\n");
	for (cnt = 0; ;cnt++)
	{
		result = f_readdir(&DirInf, &FileInf); 		/* ¶ÁÈ¡Ä¿Â¼Ïî£¬Ë÷Òý»á×Ô¶¯ÏÂÒÆ */
		if (result != FR_OK || FileInf.fname[0] == 0)
		{
			break;
		}

		if (FileInf.fname[0] == '.')
		{
			continue;
		}

		/* ÅÐ¶ÏÊÇÎÄ¼þ»¹ÊÇ×ÓÄ¿Â¼ */
		if (FileInf.fattrib & AM_DIR)
		{
			printf("(0x%02d)Ä¿Â¼  ", FileInf.fattrib);
		}
		else
		{
			printf("(0x%02d)ÎÄ¼þ  ", FileInf.fattrib);
		}

		f_stat(FileInf.fname, &fno);
		
		/* ´òÓ¡ÎÄ¼þ´óÐ¡, ×î´ó4G */
		printf(" %10d", (int)fno.fsize);


		printf("  %s\r\n", (char *)FileInf.fname);	/* ³¤ÎÄ¼þÃû */
	}
 
    /* ´òÓ¡¿¨ËÙ¶ÈÐÅÏ¢ */
    if(uSdHandle.SdCard.CardSpeed == CARD_NORMAL_SPEED)
    {
        printf("Normal Speed Card <12.5MB/S, MAX Clock < 25MHz, Spec Version 1.01\r\n");           
    }
    else if (uSdHandle.SdCard.CardSpeed == CARD_HIGH_SPEED)
    {
        printf("High Speed Card <25MB/s, MAX Clock < 50MHz, Spec Version 2.00\r\n");            
    }
    else if (uSdHandle.SdCard.CardSpeed == CARD_ULTRA_HIGH_SPEED)
    {
        printf("UHS-I SD Card <50MB/S for SDR50, DDR50 Cards, MAX Clock < 50MHz OR 100MHz\r\n");
        printf("UHS-I SD Card <104MB/S for SDR104, MAX Clock < 108MHz, Spec version 3.01\r\n");   
    }    

    
	/* Ð¶ÔØÎÄ¼þÏµÍ³ */
	 f_mount(NULL, DiskPath, 0);
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: CreateNewFile
*	¹¦ÄÜËµÃ÷: ÔÚSD¿¨´´½¨Ò»¸öÐÂÎÄ¼þ£¬ÎÄ¼þÄÚÈÝÌîÐ´¡°www.armfly.com¡±
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
int test_result = 0;
static void CreateNewFile(void)
{
	FRESULT result;
	uint32_t bw;
	char path[32];


 	/* ¹ÒÔØÎÄ¼þÏµÍ³ */
	result = f_mount(&fs, DiskPath, 0);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("¹ÒÔØÎÄ¼þÏµÍ³Ê§°Ü (%s)\r\n", FR_Table[result]);
	}

	/* ´ò¿ªÎÄ¼þ */
	sprintf(path, "%sarmfly.txt", DiskPath);
	result = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
	if (result == FR_OK)
	{
		printf("armfly.txt ÎÄ¼þ´ò¿ª³É¹¦\r\n");
	}
	else
	{	
		test_result = result; 
		printf("armfly.txt ÎÄ¼þ´ò¿ªÊ§°Ü  (%s)\r\n", FR_Table[result]);
	}

	/* Ð´Ò»´®Êý¾Ý */
	result = f_write(&file, FsWriteBuf, strlen(FsWriteBuf), &bw);
	if (result == FR_OK)
	{
		printf("armfly.txt ÎÄ¼þÐ´Èë³É¹¦\r\n");
	}
	else
	{
		printf("armfly.txt ÎÄ¼þÐ´ÈëÊ§°Ü  (%s)\r\n", FR_Table[result]);
	}

	/* ¹Ø±ÕÎÄ¼þ*/
	f_close(&file);

	/* Ð¶ÔØÎÄ¼þÏµÍ³ */
	f_mount(NULL, DiskPath, 0);
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: ReadFileData
*	¹¦ÄÜËµÃ÷: ¶ÁÈ¡ÎÄ¼þarmfly.txtÇ°128¸ö×Ö·û£¬²¢´òÓ¡µ½´®¿Ú
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
static void ReadFileData(void)
{
	FRESULT result;
	uint32_t bw;
	char path[64];

	
 	/* ¹ÒÔØÎÄ¼þÏµÍ³ */
	result = f_mount(&fs, DiskPath, 0);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("¹ÒÔØÎÄ¼þÏµÍ³Ê§°Ü (%s)\r\n", FR_Table[result]);
	}

	/* ´ò¿ªÎÄ¼þ */
	sprintf(path, "%sarmfly.txt", DiskPath);
	result = f_open(&file, path, FA_OPEN_EXISTING | FA_READ);
	if (result !=  FR_OK)
	{
		printf("Don't Find File : armfly.txt\r\n");
		return;
	}

	/* ¶ÁÈ¡ÎÄ¼þ */
	result = f_read(&file, FsReadBuf, sizeof(FsReadBuf), &bw);
	if (bw > 0)
	{
		FsReadBuf[bw] = 0;
		printf("\r\narmfly.txt ÎÄ¼þÄÚÈÝ : \r\n%s\r\n", FsReadBuf);
	}
	else
	{
		printf("\r\narmfly.txt ÎÄ¼þÄÚÈÝ : \r\n");
	}

	/* ¹Ø±ÕÎÄ¼þ*/
	f_close(&file);

	/* Ð¶ÔØÎÄ¼þÏµÍ³ */
	f_mount(NULL, DiskPath, 0);
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: CreateDir
*	¹¦ÄÜËµÃ÷: ÔÚSD¿¨¸ùÄ¿Â¼´´½¨Dir1ºÍDir2Ä¿Â¼£¬ÔÚDir1Ä¿Â¼ÏÂ´´½¨×ÓÄ¿Â¼Dir1_1
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
static void CreateDir(void)
{
	FRESULT result;
	char path[64]; 

	
 	/* ¹ÒÔØÎÄ¼þÏµÍ³ */
	result = f_mount(&fs, DiskPath, 0);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("¹ÒÔØÎÄ¼þÏµÍ³Ê§°Ü (%s)\r\n", FR_Table[result]);
	}

	/* ´´½¨Ä¿Â¼/Dir1 */
	sprintf(path, "%sDir1", DiskPath);
	result = f_mkdir(path);
	if (result == FR_OK)
	{
		printf("f_mkdir Dir1 Ok\r\n");
	}
	else if (result == FR_EXIST)
	{
		printf("Dir1 Ä¿Â¼ÒÑ¾­´æÔÚ(%d)\r\n", result);
	}
	else
	{
		printf("f_mkdir Dir1 Ê§°Ü (%s)\r\n", FR_Table[result]);
		return;
		
	}

	/* ´´½¨Ä¿Â¼/Dir2 */
	sprintf(path, "%sDir2", DiskPath);
	result = f_mkdir(path);
	if (result == FR_OK)
	{
		printf("f_mkdir Dir2 Ok\r\n");
	}
	else if (result == FR_EXIST)
	{
		printf("Dir2 Ä¿Â¼ÒÑ¾­´æÔÚ(%d)\r\n", result);
	}
	else
	{
		printf("f_mkdir Dir2 Ê§°Ü (%s)\r\n", FR_Table[result]);
		return;
	}

	/* ´´½¨×ÓÄ¿Â¼ /Dir1/Dir1_1	   ×¢Òâ£º´´½¨×ÓÄ¿Â¼Dir1_1Ê±£¬±ØÐëÏÈ´´½¨ºÃDir1 */
	sprintf(path, "%sDir1/Dir1_1", DiskPath);
	result = f_mkdir(path); /* */
	if (result == FR_OK)
	{
		printf("f_mkdir Dir1_1 ³É¹¦\r\n");
	}
	else if (result == FR_EXIST)
	{
		printf("Dir1_1 Ä¿Â¼ÒÑ¾­´æÔÚ (%d)\r\n", result);
	}
	else
	{
		printf("f_mkdir Dir1_1 Ê§°Ü (%s)\r\n", FR_Table[result]);
		return;
	}

	/* Ð¶ÔØÎÄ¼þÏµÍ³ */
	f_mount(NULL, DiskPath, 0);
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: DeleteDirFile
*	¹¦ÄÜËµÃ÷: É¾³ýSD¿¨¸ùÄ¿Â¼ÏÂµÄ armfly.txt ÎÄ¼þºÍ Dir1£¬Dir2 Ä¿Â¼
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/
static void DeleteDirFile(void)
{
	FRESULT result;
	uint8_t i;
	char path[64]; 
	
 	/* ¹ÒÔØÎÄ¼þÏµÍ³ */
	result = f_mount(&fs, DiskPath, 0);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("¹ÒÔØÎÄ¼þÏµÍ³Ê§°Ü (%s)\r\n", FR_Table[result]);
	}
	
	/* É¾³ýÄ¿Â¼/Dir1 ¡¾ÒòÎª»¹´æÔÚÄ¿Â¼·Ç¿Õ£¨´æÔÚ×ÓÄ¿Â¼)£¬ËùÒÔÕâ´ÎÉ¾³ý»áÊ§°Ü¡¿*/
	sprintf(path, "%sDir1", DiskPath);
	result = f_unlink(path);
	if (result == FR_OK)
	{
		printf("É¾³ýÄ¿Â¼Dir1³É¹¦\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("Ã»ÓÐ·¢ÏÖÎÄ¼þ»òÄ¿Â¼ :%s\r\n", "/Dir1");
	}
	else
	{
		printf("É¾³ýDir1Ê§°Ü(´íÎó´úÂë = %d) ÎÄ¼þÖ»¶Á»òÄ¿Â¼·Ç¿Õ\r\n", result);
	}

	/* ÏÈÉ¾³ýÄ¿Â¼/Dir1/Dir1_1 */
	sprintf(path, "%sDir1/Dir1_1", DiskPath);
	result = f_unlink(path);
	if (result == FR_OK)
	{
		printf("É¾³ý×ÓÄ¿Â¼/Dir1/Dir1_1³É¹¦\r\n");
	}
	else if ((result == FR_NO_FILE) || (result == FR_NO_PATH))
	{
		printf("Ã»ÓÐ·¢ÏÖÎÄ¼þ»òÄ¿Â¼ :%s\r\n", "/Dir1/Dir1_1");
	}
	else
	{
		printf("É¾³ý×ÓÄ¿Â¼/Dir1/Dir1_1Ê§°Ü(´íÎó´úÂë = %d) ÎÄ¼þÖ»¶Á»òÄ¿Â¼·Ç¿Õ\r\n", result);
	}

	/* ÏÈÉ¾³ýÄ¿Â¼/Dir1 */
	sprintf(path, "%sDir1", DiskPath);
	result = f_unlink(path);
	if (result == FR_OK)
	{
		printf("É¾³ýÄ¿Â¼Dir1³É¹¦\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("Ã»ÓÐ·¢ÏÖÎÄ¼þ»òÄ¿Â¼ :%s\r\n", "/Dir1");
	}
	else
	{
		printf("É¾³ýDir1Ê§°Ü(´íÎó´úÂë = %d) ÎÄ¼þÖ»¶Á»òÄ¿Â¼·Ç¿Õ\r\n", result);
	}

	/* É¾³ýÄ¿Â¼/Dir2 */
	sprintf(path, "%sDir2", DiskPath);
	result = f_unlink(path);
	if (result == FR_OK)
	{
		printf("É¾³ýÄ¿Â¼ Dir2 ³É¹¦\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("Ã»ÓÐ·¢ÏÖÎÄ¼þ»òÄ¿Â¼ :%s\r\n", "/Dir2");
	}
	else
	{
		printf("É¾³ýDir2 Ê§°Ü(´íÎó´úÂë = %d) ÎÄ¼þÖ»¶Á»òÄ¿Â¼·Ç¿Õ\r\n", result);
	}

	/* É¾³ýÎÄ¼þ armfly.txt */
	sprintf(path, "%sarmfly.txt", DiskPath);
	result = f_unlink(path);
	if (result == FR_OK)
	{
		printf("É¾³ýÎÄ¼þ armfly.txt ³É¹¦\r\n");
	}
	else if (result == FR_NO_FILE)
	{
		printf("Ã»ÓÐ·¢ÏÖÎÄ¼þ»òÄ¿Â¼ :%s\r\n", "armfly.txt");
	}
	else
	{
		printf("É¾³ýarmfly.txtÊ§°Ü(´íÎó´úÂë = %d) ÎÄ¼þÖ»¶Á»òÄ¿Â¼·Ç¿Õ\r\n", result);
	}

	/* É¾³ýÎÄ¼þ speed1.txt */
	for (i = 0; i < 20; i++)
	{
		sprintf(path, "%sSpeed%02d.txt", DiskPath, i);/* Ã¿Ð´1´Î£¬ÐòºÅµÝÔö */	
		result = f_unlink(path);
		if (result == FR_OK)
		{
			printf("É¾³ýÎÄ¼þ%s³É¹¦\r\n", path);
		}
		else if (result == FR_NO_FILE)
		{
			printf("Ã»ÓÐ·¢ÏÖÎÄ¼þ:%s\r\n", path);
		}
		else
		{
			printf("É¾³ý%sÎÄ¼þÊ§°Ü(´íÎó´úÂë = %d) ÎÄ¼þÖ»¶Á»òÄ¿Â¼·Ç¿Õ\r\n", path, result);
		}
	}

	/* Ð¶ÔØÎÄ¼þÏµÍ³ */
	f_mount(NULL, DiskPath, 0);
}

/*
*********************************************************************************************************
*	º¯ Êý Ãû: WriteFileTest
*	¹¦ÄÜËµÃ÷: ²âÊÔÎÄ¼þ¶ÁÐ´ËÙ¶È
*	ÐÎ    ²Î£ºÎÞ
*	·µ »Ø Öµ: ÎÞ
*********************************************************************************************************
*/

static void WriteFileTest(void)
{
	FRESULT result;
	char path[64]; 
	uint32_t bw;
	uint32_t i,k;
	uint32_t runtime1,runtime2,timelen;
	uint8_t err = 0;
	static uint8_t s_ucTestSn = 0;

	
	for (i = 0; i < sizeof(g_TestBuf); i++)
	{
		g_TestBuf[i] = (i / 512) + '0';
	}

  	/* ¹ÒÔØÎÄ¼þÏµÍ³ */
	result = f_mount(&fs, DiskPath, 0);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("¹ÒÔØÎÄ¼þÏµÍ³Ê§°Ü (%s)\r\n", FR_Table[result]);
	}

	/* ´ò¿ªÎÄ¼þ */
	sprintf(path, "%sSpeed%02d.txt", DiskPath, s_ucTestSn++); /* Ã¿Ð´1´Î£¬ÐòºÅµÝÔö */	
	result = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);

	/* Ð´Ò»´®Êý¾Ý */
	printf("¿ªÊ¼Ð´ÎÄ¼þ%s %dKB ...\r\n", path, TEST_FILE_LEN / 1024);
	
	runtime1 = bsp_GetRunTime();	/* ¶ÁÈ¡ÏµÍ³ÔËÐÐÊ±¼ä */
	for (i = 0; i < TEST_FILE_LEN / BUF_SIZE; i++)
	{
		result = f_write(&file, g_TestBuf, sizeof(g_TestBuf), &bw);
		if (result == FR_OK)
		{
			if (((i + 1) % 8) == 0)
			{
				printf(".");
			}
		}
		else
		{
			err = 1;
			printf("%sÎÄ¼þÐ´Ê§°Ü\r\n", path);
			break;
		}
	}
	runtime2 = bsp_GetRunTime();	/* ¶ÁÈ¡ÏµÍ³ÔËÐÐÊ±¼ä */
	
	if (err == 0)
	{
		timelen = (runtime2 - runtime1);
		printf("\r\n  Ð´ºÄÊ± : %dms   Æ½¾ùÐ´ËÙ¶È : %dB/S (%dKB/S)\r\n",
			timelen,
			(TEST_FILE_LEN * 1000) / timelen,
			((TEST_FILE_LEN / 1024) * 1000) / timelen);
	}

	f_close(&file);		/* ¹Ø±ÕÎÄ¼þ*/


	/* ¿ªÊ¼¶ÁÎÄ¼þ²âÊÔ */
	result = f_open(&file, path, FA_OPEN_EXISTING | FA_READ);
	if (result !=  FR_OK)
	{
		printf("Ã»ÓÐÕÒµ½ÎÄ¼þ: %s\r\n", path);
		return;
	}

	printf("¿ªÊ¼¶ÁÎÄ¼þ %dKB ...\r\n", TEST_FILE_LEN / 1024);
	
	runtime1 = bsp_GetRunTime();	/* ¶ÁÈ¡ÏµÍ³ÔËÐÐÊ±¼ä */
	for (i = 0; i < TEST_FILE_LEN / BUF_SIZE; i++)
	{
		result = f_read(&file, g_TestBuf, sizeof(g_TestBuf), &bw);
		if (result == FR_OK)
		{
			if (((i + 1) % 8) == 0)
			{
				printf(".");
			}

			/* ±È½ÏÐ´ÈëµÄÊý¾ÝÊÇ·ñÕýÈ·£¬´ËÓï¾ä»áµ¼ÖÂ¶Á¿¨ËÙ¶È½á¹û½µµÍµ½ 3.5MBytes/S */
			for (k = 0; k < sizeof(g_TestBuf); k++)
			{
				if (g_TestBuf[k] != (k / 512) + '0')
				{
				  	err = 1;
					printf("Speed1.txt ÎÄ¼þ¶Á³É¹¦£¬µ«ÊÇÊý¾Ý³ö´í\r\n");
					break;
				}
			}
			if (err == 1)
			{
				break;
			}
		}
		else
		{
			err = 1;
			printf("Speed1.txt ÎÄ¼þ¶ÁÊ§°Ü\r\n");
			break;
		}
	}

	runtime2 = bsp_GetRunTime();	/* ¶ÁÈ¡ÏµÍ³ÔËÐÐÊ±¼ä */
	
	if (err == 0)
	{
		timelen = (runtime2 - runtime1);
		printf("\r\n  ¶ÁºÄÊ± : %dms   Æ½¾ù¶ÁËÙ¶È : %dB/S (%dKB/S)\r\n", timelen,
			(TEST_FILE_LEN * 1000) / timelen, ((TEST_FILE_LEN / 1024) * 1000) / timelen);
	}

	/* ¹Ø±ÕÎÄ¼þ*/
	f_close(&file);

	/* Ð¶ÔØÎÄ¼þÏµÍ³ */
	f_mount(NULL, DiskPath, 0);
}

/***************************** °²¸»À³µç×Ó www.armfly.com (END OF FILE) *********************************/
