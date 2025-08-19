#ifndef __FONTS_H
#define __FONTS_H

#include <stdint.h>


// ������ؽṹ����
typedef struct _pFont
{    
	const uint8_t 		*pTable;  		//	��ģ�����ַ
	uint16_t 			Width; 		 	//	�����ַ�����ģ���
	uint16_t 			Height; 			//	�����ַ�����ģ����
	uint16_t 			Sizes;	 		//	�����ַ�����ģ���ݸ���
	uint16_t				Table_Rows;		// �ò���ֻ�к�����ģ�õ�����ʾ��ά������д�С
} pFONT;


/*------------------------------------ �������� ---------------------------------------------*/

// extern	pFONT	CH_Font12 ;		//	1212����
// extern	pFONT	CH_Font16 ;    //	1616����
// extern	pFONT	CH_Font20 ;    //	2020����
// extern	pFONT	CH_Font24 ;    //	2424����
// extern	pFONT	CH_Font32 ;    //	3232����


/*------------------------------------ ASCII���� ---------------------------------------------*/

// extern pFONT ASCII_Font32;		// 3216 ����
// extern pFONT ASCII_Font24;		// 2412 ����
// extern pFONT ASCII_Font20; 	// 2010 ����
// extern pFONT ASCII_Font16; 	// 1608 ����
extern pFONT ASCII_Font12; 	// 1206 ����

#endif 
 
