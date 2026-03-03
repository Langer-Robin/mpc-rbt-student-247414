#!/bin/bash
# 1. Načtení základního prostředí
source /opt/ros/humble/setup.bash

# 2. RUČNÍ EXPORT CESTY KE KNIHOVNÁM (tohle vyřeší tu chybu)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib

# 3. Nastavení logování
export LOG_LEVEL=2

# 4. Spuštění s absolutními cestami
/home/student/mpc-rbt-student-247414/build/sender_node /home/student/mpc-rbt-student-247414/config.json
