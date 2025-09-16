#!/bin/bash

# 모든 xterm 프로세스에 SIGTERM (정상 종료 요청)
pkill -TERM xterm

# 모든 ros2 프로세스를 강제 종료 (SIGKILL)
pkill -9 ros2

# 강제 종료가 필요하면 아래 명령 사용 (주석 해제 후 사용)
# pkill -9 xterm

