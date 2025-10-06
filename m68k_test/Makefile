# Makefile
.RECIPEPREFIX = >

#previously used
#    m68k-linux-gnu-gcc-10 -c *.c

lib.o: lib.c
>   m68k-linux-gnu-gcc-10 -W -c lib.c

main.o: main.c
>   m68k-linux-gnu-gcc-10 -c main.c

result.elf: main.o lib.o
#>   m68k-linux-gnu-ld -o result.elf -t *.o -lc
>   m68k-linux-gnu-ld -o result.elf *.o -lc -O0 -t -Map map.txt -T *.ld
>   m68k-linux-gnu-objcopy -O ihex result.elf result.hex

disasm:
>   m68k-linux-gnu-objdump -D result.elf

clean:
>   rm -rf *.o *.elf *.hex
