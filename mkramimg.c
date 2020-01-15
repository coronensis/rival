/* ram_image.c by Steve Rhoads 11/7/05
 * This program take the ram_xilinx.vhd file as input
 * and the code.bin file as input.
 * It then creates ram_image.vhd as output with the
 * initialization vectors set to the contents of code.bin.

 UPDATED: 09/07/10 Olivier Rinaudo (orinaudo@gmail.com)
 new behaviour: 8KB expandable to 64KB of internal RAM
 to be used with new ram_image.vhd enabling expandable
 internal ram.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>

/* 1MB buffer */
#define BUF_SIZE (1024*1024)
/* we have 1 block composed by 4 RAMB16_S9 instance (one per byte) */
/* each RAMB has 64 hex rows -> max=4*64 */
#define RAM_SPLIT (4)
#define RAM_ROWS (64)
#define RAM_ROWS_TOTAL (RAM_SPLIT*RAM_ROWS)
#define RAM_DWORDPERROW (8)


int main(int argc, char *argv[])
{
   FILE *file;
   int i, iposinrow, iblock, irowinsplit, index, size, count;
   char *buf, *ptr, *ptr_list[RAM_ROWS_TOTAL], text[80];
   unsigned int *code;

   if(argc < 3)
   {
      printf("Usage: ram_image <code.bin> <ram.vhd>\n");
      return 0;
   }

   buf = (char*)malloc(BUF_SIZE);
   code = (unsigned int*)malloc(BUF_SIZE);
   memset (code, 0, BUF_SIZE);

   /* Read ram_xilinx.vhd */
   file = fopen(argv[2], "rb");
   if(file == NULL)
   {
      printf("Can't open '%s'!\n", argv[2]);
      return -1;
   }
   size = fread(buf, 1, BUF_SIZE, file);
   fclose(file);

   /* Read code.bin */
   file = fopen(argv[1], "rb");
   if(file == NULL)
   {
      printf("Can't open '%s'!\n", argv[1]);
      return -1;
   }

   /* store DWORDs in code buffer */
   for(count = 0; count < RAM_ROWS_TOTAL*RAM_DWORDPERROW; ++count)
   {
      if(feof(file))
      {
         count--;
         break;
      }
      fread(&code[count], sizeof (uint32_t), 1, file);
      code[count] = htonl(code[count]);
   }
   fclose(file);

   count = (RAM_ROWS_TOTAL*RAM_DWORDPERROW) - 1;

   /* Find 'INIT_00 => X"' */

   /* start at buf, then seek next occurence */
   ptr = buf;
   for(i = 0; i < RAM_ROWS_TOTAL; ++i)
   {
      sprintf(text, "INIT_%2.2X => X\"", i % RAM_ROWS);
      ptr = strstr(ptr, text);
      if(ptr == NULL)
      {
         printf("ERROR: Can't find '%s', block %d, instance %d in '%s'!\n",
            text, (i/(RAM_SPLIT*RAM_ROWS)),
            (i%(RAM_SPLIT*RAM_ROWS))/RAM_ROWS, argv[2]);
         return -1;
      }
      ptr_list[i] = ptr + strlen(text);
   }

   /* Modify vhdl source code */
   iposinrow = RAM_DWORDPERROW*8-2; /* start filling from end of line */
   iblock = 0;
   irowinsplit = 0;
   for(i = 0; i < count; ++i)
   {
      sprintf(text, "%8.8x", code[i]);
      index = iblock*RAM_ROWS*RAM_SPLIT+irowinsplit;

      ptr_list[index][iposinrow]              = text[0];
      ptr_list[index][iposinrow+1]            = text[1];
      ptr_list[index+RAM_ROWS][iposinrow]     = text[2];
      ptr_list[index+RAM_ROWS][iposinrow+1]   = text[3];
      ptr_list[index+RAM_ROWS*2][iposinrow]   = text[4];
      ptr_list[index+RAM_ROWS*2][iposinrow+1] = text[5];
      ptr_list[index+RAM_ROWS*3][iposinrow]   = text[6];
      ptr_list[index+RAM_ROWS*3][iposinrow+1] = text[7];
      iposinrow -= 2;
      if(iposinrow < 0)
      {
        iposinrow = RAM_DWORDPERROW*8-2; /* reset row */
        irowinsplit++;
        if (irowinsplit>RAM_ROWS-1)
        {
          irowinsplit = 0;
          iblock++;
        }
      }
   }

   /* Write ram_image.vhd */
   file = fopen(argv[2], "wb");
   if(file == NULL)
   {
      printf("Can't write '%s'!\n", argv[2]);
      return -1;
   }
   fwrite(buf, 1, size, file);
   fclose(file);
   free(buf);
   free(code);
   return 0;
}
