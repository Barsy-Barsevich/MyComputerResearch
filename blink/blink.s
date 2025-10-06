
;        .INCLUDE    "AVR_includes/m328Pdef.inc"

        .DEVICE     ATmega328P        
        .ORG        $0000
        
START:
        LDI         R16,$08
        OUT         $3E,R16
        LDI         R16,$FF
        OUT         $3D,R16

        SBI         $04,5
LOOP:
        SBI         $05,5
        RCALL       DELAY
        CBI         $05,5
        RCALL       DELAY
        RJMP        LOOP
        
DELAY:
        LDI         R17,$FF
        LDI         R18,$FF
        LDI         R19,30
DELAY_CYCLE:
        SUBI        R17,1
        SBCI        R18,0
        SBCI        R19,0
        BRCC        DELAY_CYCLE
        RET
