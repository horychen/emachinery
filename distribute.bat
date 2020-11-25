RMDIR /Q/S build
RMDIR /Q/S dist
RMDIR /Q/S emachinery.egg-info
python setup.py sdist bdist_wheel
pause