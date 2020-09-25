
#!/bin/bash

ls
WS="/root/target_ws"

if [[ ! -z $(find $WS -type f -name '*.gcda') ]]; then
    gcovr -r $WS --exclude-directories test --xml-pretty > coverage_cpp.xml
fi
#gcovr -r . --exclude-directories test

# Actually upload coverage information
#bash <(curl -s https://codecov.io/bash) -s "$ws/build/robot_calibration/"
coveralls --exclude lib --exclude tests --gcov-options '\-lp'