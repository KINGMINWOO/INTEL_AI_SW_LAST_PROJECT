#!/bin/bash
# Qt 앱을 X11(xcb) 환경에서 실행하도록 하는 스크립트

export QT_QPA_PLATFORM=xcb
./Tab1_control   # 실제 Qt 실행 파일 이름으로 바꾸세요
