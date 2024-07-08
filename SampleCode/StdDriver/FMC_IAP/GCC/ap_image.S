;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* @copyright SPDX-License-Identifier: Apache-2.0															*/
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/

	.syntax	unified
	.arch	armv8-m.base

	.globl	loaderImage1Base
	.globl	loaderImageLimit

	.align	4
 
	.text
        
loaderImage1Base:
    .incbin  "../../LDROM_iap/LDROM_iap.bin"
loaderImageLimit:
    .space   4
    
    .end
   
