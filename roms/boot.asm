	ORG 0000h	; start at 0x0000

RST00:  di		; disable interrupts
	jp bootstrap
	nop
	nop
	nop
	nop		; pad to address 0x0008

RST08:	jp TX
	nop
	nop
	nop
	nop
	nop

RST10:	jp getc
	nop
	nop
	nop
	nop
	nop

RST18:	jp pollc
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

RST38:	reti

TX:	push af
txbusy:	in a,($80)	; read serial status
	bit 1,a		; check status bit 1
	jr z, txbusy	; loop if zero (serial is busy)
	pop af
	out ($81), a	; transmit the character
	ret

bootstrap:  
	ld hl,$FFF9	; stack initialization
	ld sp,hl

	ld a, $96	; Initialize ACIA
	out ($80),a

	di

	jp start		


;@ Follows additional functions to interact with rc2014 hardware
;@
;@---------------------------------------------------------------------
;@ getc
;@
;@  wait for system UART and return the received character in HL
;@
;@---------------------------------------------------------------------
getc:
	push af
waitch:     
	in a, ($80)
	bit 0, a
	jr z, waitch
	in a, ($81)
	ld h, 0
	ld l, a
	pop af
	ret


;@---------------------------------------------------------------------
;@ putc
;@
;@ output the byte in register L to system UART
;@
;@---------------------------------------------------------------------
putc:
	ld a, l
	rst $08
	ret


;@---------------------------------------------------------------------
;@ pollc
;@
;@ polls the uart receive buffer status and
;@ returns the result in the register L:
;@   L=0 : no data available
;@   L=1 : data available
;@
;@---------------------------------------------------------------------
pollc:
	ld l, 0
	in a, ($80)
	bit 0, a
	ret z
	ld l, 1
	ret


;@---------------------------------------------------------------------
;@ inp
;@
;@ reads a byte from port l and returns the results in l
;@
;@---------------------------------------------------------------------
inp:
	push bc
	ld c, l
	in b, (c)
	ld l, b
	pop bc
	ret


;@---------------------------------------------------------------------
;@ outp
;@
;@ writes register l to port h
;@
;@---------------------------------------------------------------------
outp:
	push bc
	ld c, h
	ld b, l
	out (c), b
	pop bc
	ret

start:
	ld hl, msg
	call print_string
loop:
	call getc		; Wait for a character
	ld a, l
	push af
	rst $08			; print it
	call newline		; print newline
	pop af
	cp 'r'
	jp nz, cont1
	call print_regs
cont1:
	cp 'm'
	jr nz, cont2
	ld hl, 0
	ld bc, 16
	call print_mem
cont2:
	cp '0'
	jr nz, loop
	jp 0
	jr loop

print_string:
        ld a, (hl)
        cp 255
        ret z
        inc hl
        rst $08
        jp print_string

delay:
	dec bc
	ld a, b
	or c
	ret z
	jr delay

print_regs:			; print AF BC DE HL IX IY in hex
	push af
	push iy
	push ix
	push hl
	push de
	push bc
	push af
	pop hl			; get AF
	call print_hex
	ld a, ' '
	rst $08
	pop hl			; get BC
	call print_hex
	ld a, ' '
	rst $08
	pop hl			; get DE
	call print_hex
	ld a, ' '
	rst $08
	pop hl			; get HL
	call print_hex
	ld a, ' '
	rst $08
	pop hl			; get IX
	call print_hex
	ld a, ' '
	rst $08
	pop hl			; get IY
	call print_hex
	call newline
	pop af
	ret

print_hex:			; print hl in hex
	ld c, h
	call out_hex
	ld c, l
out_hex:
	ld a, c
	rra
	rra
	rra
	rra
	call conv
	ld a, c
conv:
	and $0f
	add a, $90
	daa
	adc a, $40
	daa
	rst $08
	ret

print_hex8:			; print c in hex
	ld a, c
	rra
	rra
	rra
	rra
	call conv
	ld a, c
	call conv
	ret

newline:
	ld a, 13
	rst $08
	ld a, 10
	rst $08
	ret

print_mem:			; Print memory in hex from hl, bc bytes
	push bc
	call print_hex
	ld a, ' '
	rst $08
	pop bc
pmloop:
	dec bc
	ld a, b
	or c
	jr z, pmexit
	push bc
	ld c, (hl)
	inc hl
	call print_hex8
	ld a, ' '
	rst $08
	pop bc
	jr pmloop
pmexit:
	call newline
	ret

msg:
        db 'Z80 monitor', 13, 10, 255

