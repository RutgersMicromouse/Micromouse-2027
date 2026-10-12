@echo off
rem The simulation lab from Windows: passes its arguments to sim/lab/lab.py under WSL.
rem   lab.bat list
rem   lab.bat run baseline weak_r45 --trials 5
rem   lab.bat compare fan_off fan_on --set friction=0.4
rem   lab.bat sweep mass_kg 0.08,0.12,0.2,0.3
rem   lab.bat regression
rem   lab.bat experiment brake
wsl.exe -d Arch -- bash -lc "cd /mnt/c/Users/jstnp/Micromouse-2027 && python3 sim/lab/lab.py %*"
