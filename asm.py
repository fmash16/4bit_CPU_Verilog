#!/usr/bin/env python3

# Assembler for the 4 bit CPU

inst={
     "ADD A,B"          : "0",
     "SUB A,B"          : "1",
     "XCHG B,A"         : "2",
     "RCL A"            : "3",
     "OUT A"            : "4",
     "INC A"            : "5",
     "MOV B,[ADDRESS]"  : "6",
     "MOV B"            : "7",
     "JMP ADDRESS"      : "8",
     "PUSH B"           : "9",
     "POP B"            : "a",
     "NOT A"            : "b",
     "CALL"             : "c",
     "RET"              : "d",
     "TEST A,B"         : "e",
     "HLT"              : "f"
     } 

import sys

inFile = sys.argv[1]
outFile = inFile.split(sep=".", maxsplit=2)[0] + ".hex"

with open(inFile, 'r') as f:
    lines = f.read().splitlines()

count = 0
out = []

for line in lines:
    data = "0"
    line = line.strip()

    if ':' in line:
        line = line.split(sep=":", maxsplit=2)[1].strip() # Remove block names
    if ';' in line:
        line = line.split(sep=";", maxsplit=2)[0].strip() # Remove comments

    if ',' in line:
        tmp = line.split(sep=",", maxsplit=2)[1].strip()  # Seperate data

        if '[' in tmp:
            tmp.strip("[]")
            line = "MOV B,[ADDRESS]"

        if tmp.isnumeric():
            data = tmp 
            line = line.split(sep=",", maxsplit=2)[0].strip()

    if "CALL" in line:
        data = line.split()[1]
        line = line.split()[0]

    out.append(inst[line] + data)

with open(outFile, 'w') as outfile:
    for line in out:
        outfile.write("%s\n" % line)
