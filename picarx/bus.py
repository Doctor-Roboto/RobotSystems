#!/usr/bin/python3

import concurrent.futures
from readerwriterlock import rwlock

class bus():
    
    def __init__(self):
        self.lock = rwlock.RWLockWriteD()
        self.message = None
    
    def read(self):
        #print('Reading value', self.message)
        with self.lock.gen_rlock():
            message = self.message
        return message
    
    def write(self,message):
        #print('Writing value', message)
        with self.lock.gen_wlock():
            self.message = message