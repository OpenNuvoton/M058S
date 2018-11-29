;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

	.syntax	unified
	.arch	armv8-m.base

	.globl	loaderImage1Base
	.globl	loaderImageLimit

	.align	4
 
	.text
        
loaderImage1Base:
    .incbin  "..\\..\\LDROM_iap\\LDROM_iap.bin"
loaderImageLimit:
    .space   4
    
    .end
   
