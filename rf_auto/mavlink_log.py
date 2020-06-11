#!/usr/bin/env python
'''log command handling'''

import time, os

from pymavlink import mavutil


class LogDownloader():

    def __init__(self, mavlink):
        self.master: mavutil = mavlink
        self.lastLogInfo = None
        self.handleList = ['LOG_ENTRY','LOG_DATA']
        self.reset()
        self.listFinish = False
        self.downloadFinished = False

    def reset(self):
        self.download_set = set()
        self.downloadFile = None
        self.download_lognum = None
        self.download_filename = None
        self.download_start = None
        self.download_last_timestamp = None
        self.download_ofs = 0
        self.retries = 0
        self.entries = {}
        self.download_queue = []
        self.lastStatus = time.time()

    def mavlinkPacket(self, m):
        '''handle an incoming mavlink packet'''
        if m.get_type() == 'LOG_ENTRY':
            self.handleLogEntry(m)
        elif m.get_type() == 'LOG_DATA':
            self.handleLogData(m)

    def handleLogEntry(self, m):
        '''handling incoming log entry'''
        if m.time_utc == 0:
            tstring = ''
        else:
            tstring = time.ctime(m.time_utc)
        self.entries[m.id] = m
        print("Log %u  numLogs %u lastLog %u size %u %s" % (m.id, m.num_logs, m.last_log_num, m.size, tstring))
        if m.last_log_num == m.id:
            self.lastLogInfo = m
            self.listFinish = True


    def handleLogData(self, m):
        '''handling incoming log data'''
        if self.downloadFile is None:
            return
        # lose some data
        # import random
        # if random.uniform(0,1) < 0.05:
        #    print('dropping ', str(m))
        #    return
        if m.ofs != self.download_ofs:
            self.downloadFile.seek(m.ofs)
            self.download_ofs = m.ofs
        if m.count != 0:
            s = bytearray(m.data[:m.count])
            self.downloadFile.write(s)
            self.download_set.add(m.ofs // 90)
            self.download_ofs += m.count
        self.download_last_timestamp = time.time()
        if m.count == 0 or (m.count < 90 and len(self.download_set) == 1 + (m.ofs // 90)):
            dt = time.time() - self.download_start
            self.downloadFile.close()
            size = os.path.getsize(self.download_filename)
            speed = size / (1000.0 * dt)
            status = "Finished downloading %s (%u bytes %u seconds, %.1f kbyte/sec %u retries)" % (self.download_filename,
                                                                                                   size,
                                                                                                   dt, speed,
                                                                                                   self.retries)
            print(status)
            self.downloadFile = None
            self.download_filename = None
            self.download_set = set()
            self.master.mav.log_request_end_send(self.master.target_system,
                                                 self.master.target_component)
            if len(self.download_queue):
                self.logDownloadNext()
            else:
                self.downloadFinished = True
        self.updateStatus()

    def handleLogDataMissing(self):
        '''handling missing incoming log data'''
        if not self.listFinish:
            return
        if len(self.download_set) == 0:
            return
        highest = max(self.download_set)
        diff = set(range(highest)).difference(self.download_set)
        if len(diff) == 0:
            self.master.mav.log_request_data_send(self.master.target_system,
                                                       self.master.target_component,
                                                       self.download_lognum, (1 + highest) * 90, 0xffffffff)
            self.retries += 1
        else:
            num_requests = 0
            while num_requests < 20:
                start = min(diff)
                diff.remove(start)
                end = start
                while end + 1 in diff:
                    end += 1
                    diff.remove(end)
                self.master.mav.log_request_data_send(self.master.target_system,
                                                           self.master.target_component,
                                                           self.download_lognum, start * 90, (end + 1 - start) * 90)
                num_requests += 1
                self.retries += 1
                if len(diff) == 0:
                    break


    def logStatus(self):
        '''show download status'''
        if self.download_filename is None:
            print("No download")
            return
        dt = time.time() - self.download_start
        speed = os.path.getsize(self.download_filename) / (1000.0 * dt)
        m = self.entries.get(self.download_lognum, None)
        if m is None:
            size = 0
        else:
            size = m.size
        highest = 0
        if len(self.download_set):
            highest = max(self.download_set)
        diff = set(range(highest)).difference(self.download_set)

        status = "Downloading %s - %u/%u bytes %.1f kbyte/s (%u retries %u missing)" % (self.download_filename,
                                                                                        os.path.getsize(self.download_filename),
                                                                                        size,
                                                                                        speed,
                                                                                        self.retries,
                                                                                            len(diff))
        print(status)

    def logDownloadNext(self):
        if len(self.download_queue) == 0:
            self.downloadFinished = True
            return
        latest = self.download_queue.pop()
        filename = self.defaultLogFilename(latest)
        if os.path.isfile(filename) and os.path.getsize(filename) == self.entries.get(latest).to_dict()["size"]:
            print("Skipping existing %s" % (filename))
            self.logDownloadNext()
        else:
            self.logDownload(latest, filename)

    def logDownloadAll(self):
        self.downloadFinished = False
        if not self.listFinish:
            print("Please use log list first")
            return -1
        self.download_queue = sorted(self.entries, key=lambda id: self.entries[id].time_utc)
        self.logDownloadNext()
        return 0

    def logDownload(self, log_num, filename):
        '''download a log file'''
        if not self.listFinish:
            return -1
        if filename is None:
            filename = self.defaultLogFilename(log_num)
        self.downloadFinished = False
        print("Downloading log %u as %s" % (log_num, filename))
        self.download_lognum = log_num
        self.downloadFile = open(filename, "wb")
        self.master.mav.log_request_data_send(self.master.target_system,
                                                   self.master.target_component,
                                                   log_num, 0, 0xFFFFFFFF)
        self.download_filename = filename
        self.download_set = set()
        self.download_start = time.time()
        self.download_last_timestamp = time.time()
        self.download_ofs = 0
        self.retries = 0
        return 0

    def defaultLogFilename(self, log_num):
        return "log%u.bin" % log_num

    def updateStatus(self):
        '''update log download status in console'''
        now = time.time()
        if self.downloadFile is not None and now - self.lastStatus > 0.5:
            self.lastStatus = now
            self.logStatus()
            
    def idleTask(self):
        '''handle missing log data'''
        if self.download_last_timestamp is not None and time.time() - self.download_last_timestamp > 0.7:
            self.download_last_timestamp = time.time()
            self.handleLogDataMissing()
        self.updateStatus()

    def logList(self):
        self.download_set = set()
        self.listFinish = False
        self.master.mav.log_request_list_send(self.master.target_system,
                                          self.master.target_component,
                                          0, 0xffff)
    
    def downloadLatest(self, filename):
        return self.logDownload(self.lastLogInfo.id, filename)

    def eraseLog(self):
        self.master.mav.log_request_end_send(self.master.target_system,
                                             self.master.target_component)

