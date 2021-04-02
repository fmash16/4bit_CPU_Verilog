MOV B,3   
ADD A,B   ; add A and B
MOV B,6
ADD A,B
PUSH B
MOV B,2
POP B
CALL a  ; call subtraction
OUT A
HLT
subtraction:  SUB A,B
              RET
