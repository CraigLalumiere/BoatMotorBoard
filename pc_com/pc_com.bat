@echo off
pushd "%~dp0\.."
call .venv\Scripts\activate
python -m pc_com
popd
