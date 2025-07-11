RMDIR /Q/S build
RMDIR /Q/S dist
RMDIR /Q/S emachinery.egg-info
python setup.py sdist bdist_wheel

:: local test
:: pip install -e .

:: upload to test pypi
:: twine upload --repository-url https://test.pypi.org/legacy/ dist/*

:: upload to real pypi
:: twine upload dist/*

pause
