#for debugging and testing locally
git submodule update --init --recursive
./setup_drivability_checker.sh
python3 setup.py build_ext --inplace
pip install ./ --no-clean