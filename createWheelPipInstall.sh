python setup.py sdist bdist_wheel --universal

# Remove the build directory
rm -rf build

pip install dist/mocap3Dkeys-0.1.0-py2.py3-none-any.whl --force-reinstall