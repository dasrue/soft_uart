#!/usr/bin/python
import serial
import sys
import time
import threading

ser = serial.Serial( "/dev/ttySOFT0", 9600 )

running = True
soak = False

def RxThread():
  global running
  expected = ""
  try:
    while running:
      if ser.inWaiting() > 0:
        c = ser.read(1)
        if soak:
          if c != expected and expected != "" : print ( "Fail " + c + " " + hex(ord(c)) )
          expected = chr( ( ord(c) + 1 ) & 0xFF )
        elif c == ".":
          running = False
          print( "RxThread ended" )
        else:
          if c == expected or expected == "" : print( "Rx : " + c )
          else                               : print( "Rx : " + c + " FAIL" )
          expected = chr( ( ord(c) + 1 ) & 0xFF )
      else:
        time.sleep(0.1)
  except:
    print( "RxThread crashed" )
    running = False

def Echo():
  global running; running = True
  global soak;    soak    = False
  try:
    threading.Thread(target=RxThread).start()
    tx = "ABCDEFGHIJKLMNOPQRSTUVWXYZ."
    for c in tx:
      if running:
        ser.write(c)
        time.sleep(0.5)
    while running:
      time.sleep(0.5)
  except:
    print( "Except" )
    running = False

def Soak():
  global running; running = True
  global soak;    soak    = True
  try:
    threading.Thread(target=RxThread).start()
    n = 0
    while running:
      ser.write(chr(n))
      time.sleep(0.1)
      n = ( n + 1 ) & 0xFF
  except:
    print( "Except" )
    running = False

def Break():
  BRK_CHR = chr(0xBB)+chr(0)
  BRK_SET = chr(0xBB)+chr(1)
  BRK_CLR = chr(0xBB)+chr(2)
  ser.write( BRK_CHR + "U" + BRK_SET + BRK_CLR + "U" )

if len(sys.argv) <= 1:
  ser.write("U")
else:
  for n in range(1,len(sys.argv)):
    arg = sys.argv[n]
    if   arg.upper() == "BREAK"       : Break()
    elif arg.upper() == "ECHO"        : Echo()
    elif arg.upper() == "SOAK"        : Soak()
    elif arg.upper().startswith("0X") : ser.write(chr(int(arg[2:],16)))
    elif arg.upper().startswith("$")  : ser.write(chr(int(arg[1:],16)))
    elif arg.upper().startswith("%")  : ser.write(chr(int(arg[1:],2)))
    else                              : ser.write(arg)


