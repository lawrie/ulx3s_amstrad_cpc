vasmz80_oldstyle -Lnf -Lns boot.asm -Fbin -o boot.bin -L boot.lst 
python3 tomem.py boot.bin >boot.mem

