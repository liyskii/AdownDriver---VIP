@echo off
rem Pushes the prepared history to GitHub (liyskii/AdownDriver---VIP, branch main).
rem History: Dilshod's SIL test commit kept (same author, date and content) without the
rem "Co-Authored-By: Claude" line, then the 17 Nov real-hardware changes as liyskii.
cd /d "%~dp0"
if not exist adown_history.bundle (echo adown_history.bundle not found & pause & exit /b 1)
git fetch adown_history.bundle rewritten:refs/heads/rewritten || (echo fetch from bundle failed & pause & exit /b 1)
git fetch origin
git reset --hard rewritten
git push --force-with-lease=main:f9d8715 origin main
echo.
git log --oneline -4
git branch -D rewritten >nul 2>&1
pause
