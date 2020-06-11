#!/usr/bin/env python

from __future__ import print_function

import json
import queue
import threading
import time
from argparse import ArgumentParser

from pymavlink import mavutil
# FIFOキューの作成
from pymavlink.dialects.v20 import common
from pymavlink.dialects.v20.common import MAVLink_command_ack_message

from rf_auto.mavlink_log import LogDownloader

resv_queue = queue.Queue()
send_queue = queue.Queue()
debug_queue = queue.Queue()
debug = True

CMD_MODE_GUIDE = "MODE_GUIDE"
CMD_MODE_LOITER = "MODE_LOITER"
CMD_ARM = "MODE_ARM"
CMD_SET_PARAM = "CMD_SET_PARAM"
CMD_DOWNLOAD_LOG = "CMD_DOWNLOAD_LOG"
CMD_LOG_LIST = 'CMD_LOG_LIST'
CMD_EXIT = "CMD_EXIT"


def test_plan_contrl():
    file = open(args.setting_file)
    json_load = json.load(file)
    for i in json_load:
        t1 = threading.Thread(target=start_mission, args=(i,))
        t1.start()
        while t1.isAlive():
            time.sleep(1)
        time.sleep(1)
    time.sleep(1)
    send_queue.put({"exec": CMD_EXIT})


def start_mission(setting):
    time.sleep(1)
    send_queue.put({"exec": CMD_ARM})
    if "params" in setting:
        params = setting["params"]
    else:
        params = {}
    if "log_file" in setting:
        log_file = setting["log_file"]
    else:
        log_file = None
    send_queue.put({"exec": CMD_SET_PARAM, "params": params})

    while True:
        # キューからデータがなくなるまで取り出しを行う
        while not resv_queue.empty():
            m = resv_queue.get()
            if m is None:
                return
            m_type: str = m.get_type()
            time_str = "%s.%02u" % (time.strftime("%Y-%m-%d %H:%M:%S",
                                                  time.localtime(m._timestamp)),
                                    int(m._timestamp * 100.0) % 100)
            if m_type == 'STATUSTEXT':
                print("%s:%s" % (time_str, m))
                if m.text == "Disarming motors":
                    send_queue.put({"exec": CMD_LOG_LIST})
                    time.sleep(15)
                    send_queue.put({"exec": CMD_DOWNLOAD_LOG, "log_file": log_file})
            elif m_type == 'COMMAND_ACK':
                ack: MAVLink_command_ack_message = m
                print("%s:%s" % (time_str, m))
                if ack.command == common.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == common.MAV_RESULT_ACCEPTED:
                        send_queue.put({"exec": CMD_MODE_GUIDE})
                    elif ack.result == common.MAV_RESULT_FAILED:
                        send_queue.put({"exec": CMD_MODE_LOITER})
                if ack.command == common.MAV_CMD_NAV_LOITER_UNLIM:
                    send_queue.put({"exec": CMD_ARM})
            elif debug:
                print("--> %s: %s" % (time_str, m))

        time.sleep(1)

def download_log_thread():
    log_file = None
    time.sleep(1)
    send_queue.put({"exec": CMD_LOG_LIST})
    time.sleep(1)
    send_queue.put({"exec": CMD_DOWNLOAD_LOG, "log_file": log_file})
    while True:
        while not resv_queue.empty():
            m = resv_queue.get()
            if m is None:
                send_queue.put({"exec": CMD_EXIT})
                return


def mavlink_thread():
    msrc: mavutil = mavutil.mavlink_connection(args.srcport,
                                               autoreconnect=True,
                                               baud=57600,
                                               force_connected=False
                                               )
    no_print_types = ["AHRS", 'AHRS2', 'AHRS3', 'SCALED_IMU3', 'RAW_IMU', 'NAV_CONTROLLER_OUTPUT',
                      'HEARTBEAT', 'SIMSTATE', 'MEMINFO', 'SCALED_IMU2', 'RC_CHANNELS', "VFR_HUD",
                      'SYS_STATUS', 'GLOBAL_POSITION_INT', 'HOME_POSITION', 'TERRAIN_REPORT',
                      'GPS_RAW_INT', 'SCALED_PRESSURE', 'SENSOR_OFFSETS', 'POSITION_TARGET_GLOBAL_INT',
                      'LOCAL_POSITION_NED', 'POWER_STATUS', 'EKF_STATUS_REPORT', 'HWSTATUS', 'SERVO_OUTPUT_RAW',
                      'MOUNT_STATUS', 'SCALED_PRESSURE2']

    tmp_no_print = ['VIBRATION', 'SYSTEM_TIME', 'MISSION_CURRENT', 'ATTITUDE', 'TIMESYNC', 'PARAM_VALUE',
                    'BATTERY_STATUS']
    log: LogDownloader = LogDownloader(msrc)

    while True:
        m = msrc.recv_match()
        if m is not None:
            l_last_timestamp = -1
            m_type: str = m.get_type()

            if m_type == 'BAD_DATA':
                continue
            if m_type in log.handleList:
                log.mavlinkPacket(m)
                debug_queue.put(m)
                continue
            if m_type in no_print_types:
                continue
            elif m_type in tmp_no_print:
                continue

            # キューにメッセージを挿入する。
            resv_queue.put(m)
        else:
            while not send_queue.empty():
                cmd = send_queue.get()
                if cmd["exec"] == CMD_MODE_GUIDE:
                    msrc.set_mode_auto()
                elif cmd["exec"] == CMD_ARM:
                    msrc.arducopter_arm()
                elif cmd["exec"] == CMD_MODE_LOITER:
                    msrc.set_mode_loiter()
                elif cmd["exec"] == CMD_SET_PARAM:
                    for k, v in cmd["params"].items():
                        msrc.param_set_send(k, v)
                elif cmd["exec"] == CMD_LOG_LIST:
                    log.logList()
                elif cmd["exec"] == CMD_DOWNLOAD_LOG:
                    if log.downloadLatest(cmd["log_file"]) != 0:
                        send_queue.put(cmd)
                        break
                elif cmd["exec"] == CMD_EXIT:
                    return
            log.idleTask()

            time.sleep(0.01)
            if log.downloadFinished:
                log.downloadFinished = False
                log.eraseLog()
                resv_queue.put(None)


if __name__ == '__main__':
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("srcport", type=str)
    parser.add_argument("setting_file", type=str)

    args = parser.parse_args()
    t1 = threading.Thread(target=mavlink_thread)
    #t2 = threading.Thread(target=test_plan_contrl)
    t2 = threading.Thread(target=download_log_thread)
    # スレッドスタート
    t1.start()
    t2.start()
    print('started')
