@echo off
rem Opens the simulation lab: starts its server under WSL (as the other PC tests run on this
rem machine) and opens the page. The server compiles the firmware first, which takes about
rem half a minute; the page opens when that is likely done. Close this window to stop it.
rem
rem In the page: "Replay a run" plays the last simulated run, "Lab bench" changes the simulated
rem robot and runs it, "Results and progress" lists what sim\lab\lab.bat has run.
rem
rem   run_viewer.bat                                   open the lab
rem   run_viewer.bat --size 3 --random 1 --seed 7      first simulate that (any arguments of
rem                                                    sim/tests/test_firmware_drive.py), then open the lab
if not "%~1"=="" wsl.exe -d Arch -- bash -lc "cd /mnt/c/Users/jstnp/Micromouse-2027 && python3 sim/tests/test_firmware_drive.py %* --trace sim/viewer/trace.jsonl"
start "" /min cmd /c "timeout /t 30 /nobreak >nul & start http://localhost:8001/viewer/"
wsl.exe -d Arch -- bash -lc "cd /mnt/c/Users/jstnp/Micromouse-2027 && python3 sim/lab/server.py"
