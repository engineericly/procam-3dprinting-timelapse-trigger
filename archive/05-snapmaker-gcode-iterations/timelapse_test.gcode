; ---------------------------------------------------------------------------
; timelapse_test.gcode - bench-test the camera trigger with no print running
;
; Runs the exact layer-change sequence 5 times so you can watch the head hit
; the switch and confirm the FX30 fires each time. Cold and safe: it homes,
; moves in X/Y only, never touches the extruder or the heaters.
;
; Run it from Mainsail or Fluidd like any print job.
;
; Watch for: 5 shutter actuations, 5 new files on the card, and the ESP32
; serial showing "frame 1" through "frame 5" with no "trigger ignored".
; ---------------------------------------------------------------------------

G90                          ; absolute positioning
G28                          ; home all axes
M117 Timelapse trigger test

; raise a little so nothing on the bed is in the way
G1 Z10 F1200

; ---- cycle 1 ----
M117 Frame 1 of 5
G1 X250 Y265 F18000          ; approach, fast diagonal
G1 X262.8 Y265 F6000         ; press the switch -> camera triggered
M400
G4 P100                      ; hold the contact closed
G1 X255 Y265 F6000           ; back off, head out of frame
M400
G4 P2000                     ; hold still while the shot is taken
G1 X125 Y125 F18000          ; return to the middle, as a print would
G4 P3000                     ; gap between cycles

; ---- cycle 2 ----
M117 Frame 2 of 5
G1 X250 Y265 F18000
G1 X262.8 Y265 F6000
M400
G4 P100
G1 X255 Y265 F6000
M400
G4 P2000
G1 X125 Y125 F18000
G4 P3000

; ---- cycle 3 ----
M117 Frame 3 of 5
G1 X250 Y265 F18000
G1 X262.8 Y265 F6000
M400
G4 P100
G1 X255 Y265 F6000
M400
G4 P2000
G1 X125 Y125 F18000
G4 P3000

; ---- cycle 4 ----
M117 Frame 4 of 5
G1 X250 Y265 F18000
G1 X262.8 Y265 F6000
M400
G4 P100
G1 X255 Y265 F6000
M400
G4 P2000
G1 X125 Y125 F18000
G4 P3000

; ---- cycle 5 ----
M117 Frame 5 of 5
G1 X250 Y265 F18000
G1 X262.8 Y265 F6000
M400
G4 P100
G1 X255 Y265 F6000
M400
G4 P2000
G1 X125 Y125 F18000

M117 Test done - check for 5 photos
