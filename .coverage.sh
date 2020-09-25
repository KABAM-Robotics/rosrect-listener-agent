
#!/bin/bash

ls
sudo chmod +x /root/target_ws
sudo cd /root/target_ws
gcovr -r . --exclude-directories test

# Actually upload coverage information
#bash <(curl -s https://codecov.io/bash) -s "$ws/build/robot_calibration/"
coveralls --exclude lib --exclude tests --gcov-options '\-lp'